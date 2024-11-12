#!/usr/bin/env python3.10

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import threading
import rclpy
from rclpy.node import Node
import math

class geomod_encoder(Node):
    def __init__(self):
        super().__init__('geomod_encoder')
        self.heartbeat = 0
        self.platform_vel = 0.0
        self.drill_vel = 0.0
        self.drill_vel_rot = 0.0
        self.carousel_vel = 0.0
        self.needle_vel = 0.0
        self.pub = self.create_publisher( UInt8MultiArray, "can_tx", 10)
        self.sub = self.create_subscription(JointState,'geomod_commands', self.callback, 10)
        self.send_ctr = 0
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.publisher)
        self.timer = self.create_timer(0.015, self.send)
        self.get_logger().info("Geomod_Encoder Started!")
    def __del__(self):
        self.get_logger().info("Geomod_Encoder Killed!")
    def callback(self, message):
        self.platform_vel = list(message.velocity)[list(message.name).index('platform_vel')]
        self.drill_vel = list(message.velocity)[list(message.name).index('drill_vel')]
        self.drill_vel_rot = list(message.velocity)[list(message.name).index('drill_vel_rot')]
        self.carousel_vel = list(message.velocity)[list(message.name).index('carousel_vel')]
        self.needle_vel = list(message.velocity)[list(message.name).index('needle_vel')]
        self.heartbeat = 0
    def send(self):
        msg = UInt8MultiArray()
        if(self.send_ctr == 0):
            arr = np.array([self.platform_vel], dtype = np.float16)
        elif(self.send_ctr == 1):
            arr = np.array([self.drill_vel, self.drill_vel_rot], dtype = np.float16)
        elif(self.send_ctr == 2):
            arr = np.array([self.carousel_vel], dtype = np.float16)
        elif(self.send_ctr == 3):
            arr = np.array([self.needle_vel], dtype = np.float16)
        data = bytes([self.send_ctr + 31]) + arr.tobytes()
        msg.data = data
        self.pub.publish(msg)
        print(msg)
        self.send_ctr += 1
        self.send_ctr %= 4
    def publisher(self):
        self.heartbeat += 1
        if(self.heartbeat > 10):
            self.velocities = [0.] * 7
    def filter(self):
        for c in range(7):
            if (self.velocities[c] > self.velocities_filtered[c]  + 0.0035):
                self.velocities_filtered[c] += 0.003
            if (self.velocities[c] < self.velocities_filtered[c]  - 0.0035):
                self.velocities_filtered[c] -= 0.003

def main(args=None):
    rclpy.init()
    ad = geomod_encoder()
    rclpy.spin(ad)

    
    ad.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()