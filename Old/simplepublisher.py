#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('testpublisher') #initialize test publisher
        self.pubs = []
        for i in range(12):
            topicname = f'micpose{i+1}'
            self.publisher_ = self.create_publisher(Float32MultiArray, topicname, 10) #create the publisher
            self.pubs.append(self.publisher_)
        self.values = [1,2,3,4,5,6,7,8,9,10,11,12]
        self.timer = self.create_timer(0.5,self.publishcallback)

    def publishcallback(self):
        msg = Float32MultiArray()
        msg.data = []
        for i in range(12):
            msg.data.append(float(self.values[i]))
            self.pubs[i].publish(msg)
            self.get_logger().info(f"publishing {msg.data} fo mic {i+1} on topic {self.pubs[i].topic_name}")
            msg.data = []

def main(args=None):
    rclpy.init(args=args)
    testpub = MinimalPublisher()
    rclpy.spin(testpub)
    testpub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()