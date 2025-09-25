#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

# PX4 Messages
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleGlobalPosition
from px4_msgs.srv import VehicleCommand

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DroneControl(Node):
    def __init__(self):
        super().__init__('DroneControl')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        #
        vehicle_status_ = None
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_cb,
            qos_profile)
        #
        self.vehicle_gp_ = None
        self.vehicle_global_pos_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.vehicle_global_pos_cb,
            qos_profile)

    def vehicle_status_cb(self, msg):
#        self.get_logger().info(f'I heard: "{msg}"')
        self.vehicle_status_ = msg

    def vehicle_global_pos_cb(self, msg):
#        self.get_logger().info(f'I heard: "{msg}"')
        self.vehicle_gp_ = msg
    

def main(args=None):
    rclpy.init(args=args)
    node = DroneControl()
    print ("HERE3")
 #   if node.
    rclpy.spin(node)
    print ("HERE4")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
