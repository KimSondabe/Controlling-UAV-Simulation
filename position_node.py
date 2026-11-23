#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition  
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

PRINT_TIME = 0.25

class PositionNode(Node):
    def __init__(self):
        super().__init__('position_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.time = self.get_clock().now().nanoseconds * 1e-9
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.check_position = False

        self.timer = self.create_timer(PRINT_TIME, self.print_position_callback)
        self.subscription = self.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position_v1', self.listener_callback, qos_profile)
        self.get_logger().info('Position Node are running.....')

    def listener_callback(self, msg: VehicleLocalPosition):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.yaw = math.degrees(msg.heading)
        self.check_position = True
        pass


    def print_position_callback(self):
        current_time = self.get_clock().now().nanoseconds * 1e-9
        elapsed_time = current_time - self.time
        if self.check_position:
            self.get_logger().info(
                f'[Time: {elapsed_time:.2f}s |[Position] X: {self.x:.2f} m| Y: {self.y:.2f} m| Z: {self.z:.2f} m| Yaw: {self.yaw:.2f}°'
            )
        else:
            self.get_logger().info('Waiting data from PX44...')
        pass

def main(args=None):
    rclpy.init(args=args)
    node = PositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
