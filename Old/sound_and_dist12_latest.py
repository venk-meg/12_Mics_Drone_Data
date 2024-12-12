#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped


class SandDPublisher(Node):
    def __init__(self):
        super().__init__('soundanddist12_publisher') #initialize publisher
        self.publisher_distances = self.create_publisher(Float32MultiArray, 'mics/distances', 10) #create the pubisher
        self.publisher_db = self.create_publisher(Float32MultiArray, 'mics/db', 10) #create the pubisher
        self.reference_amplitude = 1.0
        self.dbval = []

    def publish_dist_message(self,poses,tellopos):        
        if poses:
            msg = Float32MultiArray()
            msg.data = []
            for pose in poses:
                dist = np.linalg.norm(tellopos - pose) #csv writing data
                #add distance to db calculation 
                # db = pose[0]*10 #placeholder calculation
                msg.data.append(float(dist)) 
            
            self.publisher_distances.publish(msg)
            self.get_logger().info(f"PUBLISHING Distance = {msg.data}")

    def publish_db_message(self,audiodatalist):
        if audiodatalist:
            msg = Float32MultiArray()
            msg.data = []
            for audiodata in audiodatalist:
                rms_amplitude = np.sqrt(np.mean(np.square(audiodata)))
                db = 20 * np.log10(rms_amplitude / self.reference_amplitude) #csv writing data
                msg.data.append(float(db)) 
            
            self.publisher_db.publish(msg)
            self.get_logger().info(f"PUBLISHING DB = {msg.data}")


class SandDSubscriber(Node):
    def __init__(self, pub):
        super().__init__('position_subscriber') #initialize subscriber
        self.pub = pub
        self.pos = []
        self.tellopos = None
        self.numberofmics = 12
        self.returnpos = []
        tellotopic = '/vrpn_client_node/Tello/pose'
        self.tellosubscription = self.create_subscription(PoseStamped,tellotopic,self.tellopositionupdate,10)
        self.tellosubscription
        for i in range(self.numberofmics):
            mictopicname = f'/vrpn_client_node/Microphone{i+1}/pose'
            self.subscription = self.create_subscription(PoseStamped,mictopicname,self.positionupdate,10)
            self.subscription
    
    def positionupdate(self,msg):
        message = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
        self.pos.append(message)
        if len(self.pos) == self.numberofmics:                
            self.get_logger().info(f"subscribed to all 12 {self.pos}")
            self.pub.publish_dist_message(self.pos,self.tellopos)
            self.pos = []

    def tellopositionupdate(self,msg):
        self.tellopos = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
        self.get_logger().info(f"tellopos = {self.tellopos}")

class AudioSubscriber(Node):
    def __init__(self,pub):
        super().__init__('audio_db_subscriber')
        self.pub = pub
        self.audiodatalist = []
        self.deviceindx = [0,1,11,12,13,14,15,16,17,18,19,20]
        for device_index in self.deviceindx:
            self.audio_sub = self.create_subscription(Float32MultiArray,f'audio_device{device_index}',self.audioprocess,10)
            self.audio_sub

    def audioprocess(self,msg):
        audio_array = np.array(msg)
        audio_array = audio_array.reshape(-1)
        self.audiodatalist.append(audio_array)
        if len(self.audiodatalist) == len(self.deviceindx):
            self.pub.publish_db_message(self.audiodatalist)
            self.audiodatalist = []

def main(args=None):
    rclpy.init(args=args)
    # dbmic = [1,2,3,4,5,6,7,8,9,10,11,12]#to test publisher
    pub = SandDPublisher()
    sub = SandDSubscriber(pub)
        
    rclpy.spin(sub)
    sub.destroy_node()
    pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()