#!/usr/bin/env python3
"""Print current joint positions in yaml-ready format for a given joint list."""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

GROUPS = {
    'left arm':  ['l_arm_joint1', 'l_arm_joint2', 'l_arm_joint3',
                  'l_arm_joint4', 'l_arm_joint5', 'l_arm_joint6'],
    'right arm': ['r_arm_joint1', 'r_arm_joint2', 'r_arm_joint3',
                  'r_arm_joint4', 'r_arm_joint5', 'r_arm_joint6'],
    'waist':     ['waist_yaw_joint'],
    'head':      ['neck_yaw_joint', 'neck_pitch_joint'],
}


def main():
    rclpy.init()
    node = Node('get_joint_positions')
    msg = None

    def cb(m):
        nonlocal msg
        msg = m

    node.create_subscription(JointState, '/joint_states', cb, 10)

    while msg is None:
        rclpy.spin_once(node, timeout_sec=1.0)

    names = list(msg.name)

    for group, joints in GROUPS.items():
        missing = [j for j in joints if j not in names]
        if missing:
            print(f'[WARN] Joint non trovati: {missing}', file=sys.stderr)
        positions = [round(msg.position[names.index(j)], 4) for j in joints if j in names]
        print(f'{group}: [{", ".join(map(str, positions))}]')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
