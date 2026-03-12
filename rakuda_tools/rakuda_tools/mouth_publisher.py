#!/usr/bin/env python3
"""
mouth_publisher.py
Nodo ROS 2 — legge l'audio dal monitor sink del ReSpeaker via parec,
calcola l'RMS e lo manda all'ESP32 (display Rakuda) via USB Serial.

Dipendenze:
    pip install pyserial
    sudo apt install pulseaudio-utils   # per parec
"""

import subprocess
import numpy as np
import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node


class MouthPublisher(Node):

    # ── Parametri di default ────────────────────────────────────────────
    MONITOR      = 'alsa_output.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.iec958-stereo.monitor'
    RATE         = 16000
    CHUNK        = 1024
    SENSITIVITY  = 0.8
    NOISE_FLOOR  = 0.01
    SERIAL_PORT  = '/dev/ttyACM0'
    SERIAL_BAUD  = 115200
    SEND_HZ      = 33          # frequenza invio (ms = 1000/SEND_HZ ≈ 30ms)

    def __init__(self):
        super().__init__('mouth_publisher')

        # ── Declare parameters (override da launch o CLI) ───────────────
        self.declare_parameter('monitor',     self.MONITOR)
        self.declare_parameter('rate',        self.RATE)
        self.declare_parameter('chunk',       self.CHUNK)
        self.declare_parameter('sensitivity', self.SENSITIVITY)
        self.declare_parameter('noise_floor', self.NOISE_FLOOR)
        self.declare_parameter('serial_port', self.SERIAL_PORT)
        self.declare_parameter('serial_baud', self.SERIAL_BAUD)
        self.declare_parameter('send_hz',     self.SEND_HZ)

        monitor     = self.get_parameter('monitor').value
        rate        = self.get_parameter('rate').value
        chunk       = self.get_parameter('chunk').value
        self.sensitivity  = self.get_parameter('sensitivity').value
        self.noise_floor  = self.get_parameter('noise_floor').value
        serial_port = self.get_parameter('serial_port').value
        serial_baud = self.get_parameter('serial_baud').value
        send_hz     = self.get_parameter('send_hz').value

        self.chunk = chunk
        self.rate  = rate
        self.bytes_per_chunk = chunk * 2   # s16le = 2 byte/campione

        # ── Serial ─────────────────────────────────────────────────────
        port = self._find_esp32_port() or serial_port
        try:
            self.ser = serial.Serial(port, serial_baud,
                                     timeout=1, dtr=False, rts=False)
            self.get_logger().info(f'ESP32 connesso su {port}')
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(f'Serial non disponibile ({e})')

        # ── parec ───────────────────────────────────────────────────────
        cmd = [
            'parec',
            '--device', monitor,
            '--format=s16le',
            f'--rate={rate}',
            '--channels=1',
        ]
        try:
            self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                         stderr=subprocess.DEVNULL)
            self.get_logger().info(f'parec avviato — monitor: {monitor}')
        except FileNotFoundError:
            self.get_logger().error('parec non trovato. Installa: sudo apt install pulseaudio-utils')
            self.proc = None

        # ── Timer ───────────────────────────────────────────────────────
        period = 1.0 / send_hz
        self.timer = self.create_timer(period, self._timer_cb)

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
        if self.proc is None:
            return

        raw = self.proc.stdout.read(self.bytes_per_chunk)
        if not raw:
            self.get_logger().warn('parec ha chiuso lo stream')
            return

        data  = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
        data /= 32768.0

        rms = float(np.sqrt(np.mean(data ** 2))) * self.sensitivity * 4

        # Stop immediato sotto il noise floor
        if rms < self.noise_floor:
            rms = 0.0

        rms_byte = int(min(rms, 1.0) * 255)

        if self.ser:
            try:
                self.ser.write(f'{rms_byte}\n'.encode())
            except serial.SerialException as e:
                self.get_logger().warn(f'Errore serial: {e}')

    # ────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        if self.proc:
            self.proc.terminate()
        if self.ser:
            self.ser.close()
        super().destroy_node()


# ────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MouthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()