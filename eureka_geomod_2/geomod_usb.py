#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 
from sensor_msgs.msg import JointState
import numpy as np
import rclpy
from rclpy.node import Node
import serial
from neuron import h


class arm_usb(Node):
    def __init__(self):
        super().__init__('geomod_usb')
        self.sub = self.create_subscription(JointState, "geomod_commands", self.arm_callback, 10)
        self.sub_3 = self.create_subscription(JointState, "geomod_settings", self.settings_callback, 10)
   #     self.pub = self.create_publisher(JointState, "geomod_states", 10)
        #commands
        self.heartbeat = 1
        self.control_mode = 0
        self.power_saving = 0
        self.platform_vel = 0.0
        self.drill_vel = 0.0
        self.drill_vel_rot = 0.0
        self.carousel_vel = 0.0
        self.needle_vel = 0.0
        self.command_format = "global: heartbeat=%d, control_mode=%d, power_saving=%d\r\n\
platform: lin_vel=%.2f\r\n\
drill: lin_vel=%.2f, ang_vel=%.2f\r\n\
carousel: rot_vel=%.2f\r\n\
needle: lin_vel = %.2f\r\n\
__end__";
        self.reply_format = "Nothing to reply!\r\n\
__end__"
        flag = 0
        while flag < 1:
            try:
                self.arm = serial.Serial('/dev/geomod', 9600, timeout=1)
                flag = 1
                continue
            except serial.serialutil.SerialException:
                None
        flag = 0
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.send)
        self.send()
        self.get_logger().info("usb_geomod Started!")
    def __del__(self):
        self.get_logger().info("usb_geomod Killed!")

    def send(self):
            print(len(self.command_format))
            message = self.command_format % (self.heartbeat, self.control_mode, self.power_saving,
                                             self.platform_vel, self.drill_vel, self.drill_vel_rot, 
                                             self.carousel_vel, self.needle_vel)
            print(message)
            print(self.arm.write(bytes(message, encoding='utf8')))
            reply = self.arm.read_until(str.encode("__end__")).decode('utf-8')
            print(reply)



    def arm_callback(self,message):
        self.platform_vel = list(message.velocity)[list(message.name).index('platform_vel')]
        self.drill_vel = list(message.velocity)[list(message.name).index('drill_vel')]
        self.drill_vel_rot = list(message.velocity)[list(message.name).index('drill_vel_rot')]
        self.carousel_vel = list(message.velocity)[list(message.name).index('carousel_vel')]
        self.needle_vel = list(message.velocity)[list(message.name).index('needle_vel')]
    def settings_callback(self,data):
        self.heartbeat = data.position[list(data.name).index('heartbeat')]
        self.control_mode = data.position[list(data.name).index('control_mode')]
        self.power_saving = data.position[list(data.name).index('power_saving')]




def main(args=None):
    rclpy.init()
    usb = arm_usb()
    rclpy.spin(usb)

    
    usb.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()