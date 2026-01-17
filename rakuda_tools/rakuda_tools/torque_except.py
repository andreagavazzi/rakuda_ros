#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool
from dynamixel_interfaces.srv import SetDataToDxl
from std_msgs.msg import Float64MultiArray


class TorqueExceptNode(Node):
    def __init__(self):
        super().__init__('torque_except')

        self.declare_parameter('keep_ids', [1, 2, 3])
        self.declare_parameter('disable_all_first', True)

        # Optional: after torque setup, publish a simple 'home' to head controller (position group)
        self.declare_parameter('send_head_home', False)
        self.declare_parameter('head_home_positions', [0.0, 0.0])  # [yaw, pitch] rad

        # Where to publish home:
        # - 'commands' -> /head_controller/commands (recommended for JointGroupPositionController)
        # - 'target'   -> /head_target (if you want head_gimbal_pos to handle it)
        self.declare_parameter('head_home_topic_mode', 'commands')
        self.declare_parameter('head_home_pub_count', 10)
        self.declare_parameter('head_home_pub_rate_hz', 20.0)

        self.keep_ids = [int(x) for x in self.get_parameter('keep_ids').value]
        self.disable_all_first = bool(self.get_parameter('disable_all_first').value)

        self.send_head_home = bool(self.get_parameter('send_head_home').value)
        self.head_home_positions = [float(x) for x in self.get_parameter('head_home_positions').value]
        self.head_home_topic_mode = str(self.get_parameter('head_home_topic_mode').value)
        self.head_home_pub_count = int(self.get_parameter('head_home_pub_count').value)
        self.head_home_pub_rate_hz = float(self.get_parameter('head_home_pub_rate_hz').value)

        self.cli_all = self.create_client(SetBool, '/dynamixel_hardware_interface/set_dxl_torque')
        self.cli_set = self.create_client(SetDataToDxl, '/dynamixel_hardware_interface/set_dxl_data')

        # Publishers for "home"
        self.pub_head_commands = self.create_publisher(Float64MultiArray, '/head_controller/commands', 10)
        self.pub_head_target = self.create_publisher(Float64MultiArray, '/head_target', 10)

        self._pending = 0
        self._home_left = 0
        self._home_timer = None

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
            if self.send_head_home:
                self.get_logger().info('Torque setup complete. Publishing head home...')
                self._publish_head_home_start()
            else:
                self.get_logger().info('Done.')
                rclpy.shutdown()

    def _publish_head_home_start(self):
        if len(self.head_home_positions) != 2:
            self.get_logger().error(f"head_home_positions must be [yaw, pitch], got: {self.head_home_positions}")
            rclpy.shutdown()
            return

        self._home_left = max(1, self.head_home_pub_count)
        period = 1.0 / max(1.0, self.head_home_pub_rate_hz)

        # publish immediately and then with a timer
        self._publish_head_home_once()
        self._home_left -= 1

        if self._home_left <= 0:
            self.get_logger().info('Head home published. Fine.')
            rclpy.shutdown()
            return

        self._home_timer = self.create_timer(period, self._publish_head_home_timer_cb)

    def _publish_head_home_timer_cb(self):
        self._publish_head_home_once()
        self._home_left -= 1
        if self._home_left <= 0:
            if self._home_timer is not None:
                self._home_timer.cancel()
            self.get_logger().info('Head home published. Fine.')
            rclpy.shutdown()

    def _publish_head_home_once(self):
        yaw, pitch = self.head_home_positions
        msg = Float64MultiArray()
        msg.data = [float(yaw), float(pitch)]

        mode = self.head_home_topic_mode.lower().strip()
        if mode == 'target':
            self.pub_head_target.publish(msg)
        else:
            # default: commands
            self.pub_head_commands.publish(msg)

def main():
    rclpy.init()
    node = TorqueExceptNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
