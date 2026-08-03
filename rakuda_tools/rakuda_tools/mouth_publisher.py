#!/usr/bin/env python3
"""
mouth_publisher.py
Nodo ROS 2 — legge l'audio dal monitor sink del ReSpeaker via parec,
calcola l'RMS e lo manda all'ESP32 (display Rakuda) via USB Serial.

Caratteristiche:
  - stdout di parec in non-blocking, drain completo del pipe a ogni tick
    (niente accumulo di latenza)
  - --latency-msec per buffer piccoli lato PipeWire/PulseAudio
  - envelope follower attack/release (bocca fluida, niente sfarfallio)
  - mapping percettivo sqrt (scala piu' uniforme sul parlato)
  - invio seriale solo su variazione del valore
  - watchdog: restart automatico di parec se muore (PipeWire restart,
    device scollegato, ecc.)
  - nessuna dipendenza da numpy

Dipendenze:
    pip install pyserial
    sudo apt install pulseaudio-utils   # per parec
"""

import os
import array
import fcntl
import math
import subprocess
import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node


class MouthPublisher(Node):

    # ── Parametri di default ────────────────────────────────────────────
    MONITOR       = 'alsa_output.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.analog-stereo.monitor'
    RATE          = 16000
    CHUNK         = 512         # 32 ms @ 16 kHz, allineato al timer a 33 Hz
    SENSITIVITY   = 0.8
    NOISE_FLOOR   = 0.01
    SERIAL_PORT   = 'ttyESP32_LCD'
    SERIAL_BAUD   = 115200
    SEND_HZ       = 33          # frequenza invio (ms = 1000/SEND_HZ ≈ 30ms)
    LATENCY_MS    = 20          # latenza richiesta a parec
    ATTACK        = 0.6         # coeff. envelope in salita (0-1, alto = rapido)
    RELEASE       = 0.25        # coeff. envelope in discesa
    RESTART_SEC   = 2.0         # attesa prima di riavviare parec dopo un crash

    def __init__(self):
        super().__init__('mouth_publisher')

        # ── Declare parameters (override da launch o CLI) ───────────────
        self.declare_parameter('monitor',      self.MONITOR)
        self.declare_parameter('rate',         self.RATE)
        self.declare_parameter('chunk',        self.CHUNK)
        self.declare_parameter('sensitivity',  self.SENSITIVITY)
        self.declare_parameter('noise_floor',  self.NOISE_FLOOR)
        self.declare_parameter('serial_port',  self.SERIAL_PORT)
        self.declare_parameter('serial_baud',  self.SERIAL_BAUD)
        self.declare_parameter('send_hz',      self.SEND_HZ)
        self.declare_parameter('latency_msec', self.LATENCY_MS)
        self.declare_parameter('attack',       self.ATTACK)
        self.declare_parameter('release',      self.RELEASE)

        self.monitor      = self._find_monitor_sink() or self.get_parameter('monitor').value
        self.rate         = self.get_parameter('rate').value
        self.chunk        = self.get_parameter('chunk').value
        self.sensitivity  = self.get_parameter('sensitivity').value
        self.noise_floor  = self.get_parameter('noise_floor').value
        serial_port       = self.get_parameter('serial_port').value
        serial_baud       = self.get_parameter('serial_baud').value
        send_hz           = self.get_parameter('send_hz').value
        self.latency_msec = self.get_parameter('latency_msec').value
        self.attack       = self.get_parameter('attack').value
        self.release      = self.get_parameter('release').value

        self.bytes_per_chunk = self.chunk * 2   # s16le = 2 byte/campione

        # ── Stato ──────────────────────────────────────────────────────
        self.proc       = None
        self._fd        = None
        self._buf       = b''
        self._env       = 0.0     # envelope follower
        self._last_sent = -1      # ultimo byte inviato (forza il primo invio)
        self._restart_cooldown = 0.0

        # ── Serial ─────────────────────────────────────────────────────
        # Priorita' al device configurato (regola udev): l'auto-detect e'
        # solo fallback, perche' matcha anche altri FTDI/CDC (es. U2D2
        # del bus DYNAMIXEL) e potrebbe scrivere sul device sbagliato.
        port = serial_port
        if not port.startswith('/dev/'):
            port = '/dev/' + port
        if not os.path.exists(port):
            detected = self._find_esp32_port()
            if detected:
                self.get_logger().warn(
                    f'{port} not found, falling back to auto-detect: {detected}')
                port = detected
        try:
            self.ser = serial.Serial(port, serial_baud, timeout=1)
            self.ser.dtr = False
            self.ser.rts = False
            self.get_logger().info(f'ESP32 detected on port {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'{e}')
            raise SystemExit(1)

        # ── parec ───────────────────────────────────────────────────────
        self._start_parec()

        # ── Timer ───────────────────────────────────────────────────────
        self._period = 1.0 / send_hz
        self.timer = self.create_timer(self._period, self._timer_cb)

    # ────────────────────────────────────────────────────────────────────
    def _start_parec(self) -> bool:
        """Avvia (o riavvia) parec con stdout non-blocking."""
        cmd = [
            'parec',
            '--device', self.monitor,
            '--format=s16le',
            f'--rate={self.rate}',
            '--channels=1',
            f'--latency-msec={self.latency_msec}',
        ]
        try:
            self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                         stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            self.get_logger().error(
                'parec not found! Please install: sudo apt install pulseaudio-utils')
            self.proc = None
            self._fd = None
            return False

        self._fd = self.proc.stdout.fileno()
        fl = fcntl.fcntl(self._fd, fcntl.F_GETFL)
        fcntl.fcntl(self._fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        self._buf = b''
        self.get_logger().info(f'Receiving from {self.monitor}')
        return True

    # ────────────────────────────────────────────────────────────────────
    def _stop_parec(self):
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None
        self._fd = None
        self._buf = b''

    # ────────────────────────────────────────────────────────────────────
    def _watchdog(self):
        """Riavvia parec se e' morto, con cooldown per non fare spin-loop."""
        self._restart_cooldown -= self._period
        if self._restart_cooldown > 0:
            return
        self._restart_cooldown = self.RESTART_SEC

        self._stop_parec()
        # il device puo' essere cambiato (riconnessione USB): ricerca di nuovo
        self.monitor = self._find_monitor_sink() or self.monitor
        if self._start_parec():
            self.get_logger().info('parec restarted')

    # ────────────────────────────────────────────────────────────────────
    def _find_monitor_sink(self) -> str | None:
        """Cerca dinamicamente il monitor del ReSpeaker via pactl."""
        try:
            out = subprocess.check_output(
                ['pactl', 'list', 'sources', 'short'],
                stderr=subprocess.DEVNULL, text=True
            )
            for line in out.splitlines():
                name = line.split('\t')[1] if '\t' in line else ''
                if 'respeaker' in name.lower() or 'seeed' in name.lower():
                    if name.endswith('.monitor'):
                        self.get_logger().info(f'Auto-detected monitor: {name}')
                        return name
        except (FileNotFoundError, subprocess.CalledProcessError):
            pass
        return None

    # ────────────────────────────────────────────────────────────────────
    def _find_esp32_port(self) -> str | None:
        for port in serial.tools.list_ports.comports():
            desc = port.description.lower()
            if any(k in desc for k in ['cp210', 'ch340', 'ch341', 'ftdi',
                                        'esp32', 'silabs', 'cdc', 'jtag']):
                return port.device
        return None

    # ────────────────────────────────────────────────────────────────────
    def _timer_cb(self):
        # parec morto o mai partito -> watchdog
        if self.proc is None or self.proc.poll() is not None:
            if self.proc is not None:
                self.get_logger().warn('parec terminated, restarting...')
                self._send(0)          # chiudi la bocca nel frattempo
            self._watchdog()
            return

        # ── Drena TUTTO il pipe: impedisce l'accumulo di latenza ───────
        try:
            while True:
                data = os.read(self._fd, 65536)
                if not data:           # EOF
                    break
                self._buf += data
        except BlockingIOError:
            pass                       # pipe vuoto, ok

        if len(self._buf) < self.bytes_per_chunk:
            return                     # dati insufficienti, riprova al prossimo tick

        # ── Usa solo l'ultimo chunk, conserva la coda per il tick dopo ─
        raw = self._buf[-self.bytes_per_chunk:]
        self._buf = self._buf[-self.bytes_per_chunk:]

        # ── RMS senza numpy ────────────────────────────────────────────
        samples = array.array('h', raw)
        rms = math.sqrt(sum(s * s for s in samples) / len(samples)) / 32768.0
        rms *= self.sensitivity * 4

        if rms < self.noise_floor:
            # silenzio: chiusura immediata, niente coda di release
            self._env = 0.0
        else:
            # ── Envelope follower ──────────────────────────────────────
            # attacco rapido; release a due stadi: se il segnale crolla
            # (fine parola/frase) scende veloce, se cala poco smorza soltanto
            if rms > self._env:
                alpha = self.attack
            elif rms < self._env * 0.4:
                alpha = 0.7            # fast release: il segnale e' crollato
            else:
                alpha = self.release
            self._env += alpha * (rms - self._env)
            if self._env < self.noise_floor:
                self._env = 0.0        # snap a zero

        # ── Mapping percettivo (sqrt) ──────────────────────────────────
        value = int(math.sqrt(min(self._env, 1.0)) * 255)

        self._send(value)

    # ────────────────────────────────────────────────────────────────────
    def _send(self, value: int):
        """Invia il byte all'ESP32 solo se diverso dall'ultimo inviato."""
        if value == self._last_sent or self.ser is None:
            return
        try:
            self.ser.write(f'{value}\n'.encode())
            self._last_sent = value
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial error: {e}')

    # ────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._stop_parec()
        if self.ser:
            try:
                self.ser.write(b'-1\n')   # segnale di shutdown all'ESP32
            except serial.SerialException:
                pass
            self.ser.close()
        print(' [mouth_publisher]: Node closed, mouth disconnected!')
        super().destroy_node()


# ────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MouthPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == '__main__':
    main()
