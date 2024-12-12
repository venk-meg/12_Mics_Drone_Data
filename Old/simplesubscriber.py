#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('testsubscriber') #initialize test publisher
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback,10) #create the subscriber
        self.subscription

    def listener_callback(self,msg):
        self.get_logger().info(f"subscribing {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    testsub = MinimalSubscriber()
    rclpy.spin(testsub)
    testsub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()