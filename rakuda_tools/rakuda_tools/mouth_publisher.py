#!/usr/bin/env python3
"""
mouth_publisher.py
Nodo ROS 2 — legge l'audio dal monitor sink del ReSpeaker via parec e manda
all'ESP32 (display Rakuda) un gate binario via USB Serial: 255 = parlato,
0 = silenzio (il firmware anima barre random, serve solo on/off).

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

import rclpy
from rclpy.node import Node


class MouthPublisher(Node):

    MONITOR     = 'alsa_output.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.analog-stereo.monitor'
    RATE        = 16000
    CHUNK       = 512           # 32 ms @ 16 kHz
    NOISE_FLOOR = 0.013         # soglia RMS (raw): unica manopola di taratura
    SERIAL_PORT = '/dev/ttyESP32_LCD'
    SERIAL_BAUD = 115200
    SEND_HZ     = 33
    LATENCY_MS  = 20
    RESTART_SEC = 2.0           # cooldown restart parec

    def __init__(self):
        super().__init__('mouth_publisher')

        for name, default in [('monitor', self.MONITOR), ('rate', self.RATE),
                              ('chunk', self.CHUNK), ('noise_floor', self.NOISE_FLOOR),
                              ('serial_port', self.SERIAL_PORT),
                              ('serial_baud', self.SERIAL_BAUD),
                              ('send_hz', self.SEND_HZ),
                              ('latency_msec', self.LATENCY_MS)]:
            self.declare_parameter(name, default)
        p = lambda name: self.get_parameter(name).value

        self.monitor      = self._find_monitor_sink() or p('monitor')
        self.rate         = p('rate')
        self.noise_floor  = p('noise_floor')
        self.latency_msec = p('latency_msec')
        self.bytes_per_chunk = p('chunk') * 2   # s16le

        self.proc = None
        self._fd  = None
        self._buf = b''
        self._gate_open = False
        self._last_sent = -1
        self._cooldown  = 0.0

        try:
            self.ser = serial.Serial(p('serial_port'), p('serial_baud'), timeout=1)
            self.ser.dtr = False
            self.ser.rts = False
            self.get_logger().info(f"ESP32 on {p('serial_port')}")
        except serial.SerialException as e:
            self.get_logger().error(f'{e}')
            raise SystemExit(1)

        self._start_parec()
        self._period = 1.0 / p('send_hz')
        self.timer = self.create_timer(self._period, self._timer_cb)

    def _start_parec(self) -> bool:
        cmd = ['parec', '--device', self.monitor, '--format=s16le',
               f'--rate={self.rate}', '--channels=1',
               f'--latency-msec={self.latency_msec}']
        try:
            self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                         stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            self.get_logger().error('parec not found: sudo apt install pulseaudio-utils')
            self.proc, self._fd = None, None
            return False
        self._fd = self.proc.stdout.fileno()
        fcntl.fcntl(self._fd, fcntl.F_SETFL,
                    fcntl.fcntl(self._fd, fcntl.F_GETFL) | os.O_NONBLOCK)
        self._buf = b''
        self.get_logger().info(f'Receiving from {self.monitor}')
        return True

    def _stop_parec(self):
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc, self._fd, self._buf = None, None, b''

    def _find_monitor_sink(self) -> str | None:
        try:
            out = subprocess.check_output(['pactl', 'list', 'sources', 'short'],
                                          stderr=subprocess.DEVNULL, text=True)
            for line in out.splitlines():
                name = line.split('\t')[1] if '\t' in line else ''
                if ('respeaker' in name.lower() or 'seeed' in name.lower()) \
                        and name.endswith('.monitor'):
                    return name
        except (FileNotFoundError, subprocess.CalledProcessError):
            pass
        return None

    def _timer_cb(self):
        # watchdog: parec morto -> restart con cooldown
        if self.proc is None or self.proc.poll() is not None:
            self._send(0)
            self._cooldown -= self._period
            if self._cooldown <= 0:
                self._cooldown = self.RESTART_SEC
                self._stop_parec()
                self.monitor = self._find_monitor_sink() or self.monitor
                if self._start_parec():
                    self.get_logger().info('parec (re)started')
            return

        # drena tutto il pipe: niente accumulo di latenza
        try:
            while True:
                data = os.read(self._fd, 65536)
                if not data:
                    break
                self._buf += data
        except BlockingIOError:
            pass

        if len(self._buf) < self.bytes_per_chunk:
            return
        raw = self._buf[-self.bytes_per_chunk:]   # solo il chunk piu' recente
        self._buf = raw

        samples = array.array('h', raw)
        rms = math.sqrt(sum(s * s for s in samples) / len(samples)) / 32768.0

        # gate con isteresi: chiusura immediata sotto floor, apertura a floor*1.5
        if self._gate_open:
            self._gate_open = rms >= self.noise_floor
        else:
            self._gate_open = rms > self.noise_floor * 1.5

        self._send(255 if self._gate_open else 0)

    def _send(self, value: int):
        if value == self._last_sent or self.ser is None:
            return
        try:
            self.ser.write(f'{value}\n'.encode())
            self._last_sent = value
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial error: {e}')

    def destroy_node(self):
        self._stop_parec()
        if self.ser:
            try:
                self.ser.write(b'-1\n')   # shutdown ESP32
            except serial.SerialException:
                pass
            self.ser.close()
        print(' [mouth_publisher]: Node closed, mouth disconnected!')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MouthPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
