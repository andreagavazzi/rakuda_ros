#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool
from dynamixel_interfaces.srv import SetDataToDxl


class TorqueExceptNode(Node):
    def __init__(self):
        super().__init__('torque_except')

        self.declare_parameter('keep_ids', [2, 3])
        self.declare_parameter('disable_all_first', True)

        self.keep_ids = [int(x) for x in self.get_parameter('keep_ids').value]
        self.disable_all_first = bool(self.get_parameter('disable_all_first').value)

        self.cli_all = self.create_client(SetBool, '/dynamixel_hardware_interface/set_dxl_torque')
        self.cli_set = self.create_client(SetDataToDxl, '/dynamixel_hardware_interface/set_dxl_data')

        self._pending = 0

        # Aspetta che i service siano disponibili, poi parte
        self.timer = self.create_timer(0.2, self._tick)

    def _tick(self):
        if not self.cli_all.service_is_ready() or not self.cli_set.service_is_ready():
            self.get_logger().info('Aspetto servizi /set_dxl_torque e /set_dxl_data...')
            return

        self.timer.cancel()
        self.get_logger().info(f'Servizi ok. keep_ids={self.keep_ids}, disable_all_first={self.disable_all_first}')

        if self.disable_all_first:
            self._call_disable_all()
        else:
            self._enable_keep_ids()

    def _call_disable_all(self):
        req = SetBool.Request()
        req.data = False
        fut = self.cli_all.call_async(req)
        fut.add_done_callback(self._on_disable_all_done)

    def _on_disable_all_done(self, fut):
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f'Errore chiamando set_dxl_torque(false): {e}')
            rclpy.shutdown()
            return

        self.get_logger().info(f'set_dxl_torque(false) -> success={res.success} msg="{res.message}"')
        self._enable_keep_ids()

    def _enable_keep_ids(self):
        # Se non ci sono ID da tenere, chiudiamo
        if not self.keep_ids:
            self.get_logger().info('Nessun keep_id specificato. Fine.')
            rclpy.shutdown()
            return

        for dxl_id in self.keep_ids:
            if not (0 <= dxl_id <= 255):
                self.get_logger().error(f'ID fuori range uint8: {dxl_id}')
                continue

            req = SetDataToDxl.Request()
            req.header.stamp.sec = 0
            req.header.stamp.nanosec = 0
            req.header.frame_id = ''
            req.id = dxl_id
            req.item_name = 'Torque Enable'
            req.item_data = 1  # 1=ON

            self._pending += 1
            f = self.cli_set.call_async(req)
            f.add_done_callback(lambda ff, _id=dxl_id: self._on_enable_done(ff, _id))

        if self._pending == 0:
            self.get_logger().info('Nessuna richiesta inviata (tutti ID invalidi?). Fine.')
            rclpy.shutdown()

    def _on_enable_done(self, fut, dxl_id: int):
        try:
            res = fut.result()
            ok = bool(res.result)
        except Exception as e:
            self.get_logger().error(f'ID {dxl_id}: errore set_dxl_data: {e}')
            ok = False

        self.get_logger().info(f'ID {dxl_id}: Torque Enable=1 -> result={ok}')

        self._pending -= 1
        if self._pending <= 0:
            self.get_logger().info('Fatto.')
            rclpy.shutdown()


def main():
    rclpy.init()
    node = TorqueExceptNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

