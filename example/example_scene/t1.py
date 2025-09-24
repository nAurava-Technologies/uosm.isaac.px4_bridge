#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg._vehicle_status import VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MinimalPySubscriber(Node):
    def __init__(self):
        super().__init__('minimal_py_subscriber')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscriber_1 = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.listener_callback,
            qos_profile)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg}"')
#        self.get_logger().info(f'I heard: "{msg.data}"')
#        print(f'I heard: "{msg}"')

def main(args=None):
    rclpy.init(args=args)
    print ("HERE2")
    minimal_py_subscriber = MinimalPySubscriber()
    print ("HERE3")
#    rclpy.spin_once(minimal_py_subscriber)
    print ("HERE3a")
    rclpy.spin(minimal_py_subscriber)
    print ("HERE4")
    minimal_py_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print ("HERE")
    main()
