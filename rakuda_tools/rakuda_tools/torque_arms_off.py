#!/usr/bin/env python3
"""Disable torque on all arm Dynamixel motors."""

import rclpy
from rclpy.node import Node
from dynamixel_interfaces.srv import SetDataToDxl

ARM_IDS = [11, 12, 13, 14, 15, 16, 17, 21, 22, 23, 24, 25, 26, 27]


def main():
    rclpy.init()
    node = Node('torque_arms_off')

    cli = node.create_client(SetDataToDxl, '/dynamixel_hardware_interface/set_dxl_data')
    node.get_logger().info('Aspetto servizio...')
    cli.wait_for_service()
    node.get_logger().info(f'Disabilitazione torque su IDs {ARM_IDS}')

    for dxl_id in ARM_IDS:
        req = SetDataToDxl.Request()
        req.id = dxl_id
        req.item_name = 'Torque Enable'
        req.item_data = 0
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        res = future.result()
        ok = res.result if res else False
        node.get_logger().info(f'ID {dxl_id}: {"OK" if ok else "FAIL"}')

    node.get_logger().info('Done.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
