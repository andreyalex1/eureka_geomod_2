#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import rclpy
from rclpy.node import Node
import serial

class cwt_npkphcth_reader(Node):
    def __init__(self):
        self.nitrogen = 0
        self.phosphorus = 0
        self.potassium = 0
        self.ph = 0
        self.conductivity = 0
        self.temperature = 0
        self.moisture = 0
        try:
            self.ser = serial.Serial('/dev/needle', 4800, timeout=1)
            print(self.ser.name) 
        except serial.serialutil.SerialException:
            serial.close()
            serial.open()
        super().__init__('cwt_npkphcth_reader')
        self.pub = self.create_publisher(JointState,'needle', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publisher)
        self.get_logger().info("cwt_npkphcth_reader Started!")
    def __del__(self):
        self.get_logger().info("cwt_npkphcth_reader Killed!")
    def publisher(self):
        try:
            message = JointState()
            self.ser.write(b'\x01\x03\x00\x00\x00\x07\x04\x08')
            bt = self.ser.read(19)
            ba = bytearray(bt)
            self.moisture = float(int.from_bytes(ba[3:5], "big") / 10)
            self.temperature = float(int.from_bytes(ba[5:7], "big") / 10)
            self.conductivity = float(int.from_bytes(ba[7:9], "big") / 10)
            self.ph = float(int.from_bytes(ba[9:11], "big") / 10)
            self.nitrogen = float(int.from_bytes(ba[11:13], "big") / 10)
            self.phosphorus = float(int.from_bytes(ba[13:15], "big") / 10)
            self.potassium = float(int.from_bytes(ba[15:17], "big") / 10)
            message.name = ['nitrogen', 'phosphorus', 'potassium', 'ph', 'conductivity', 'soil_temperature', 'moisture']
            message.position = [self.nitrogen, self.phosphorus, self.potassium, self.ph, self.conductivity, self.temperature, self.moisture]
            self.pub.publish(message)
        except serial.serialutil.SerialException:
            self.get_logger().warning("No USB Connection to Needle!")
            try:
                self.ser = serial.Serial('/dev/needle', 4800, timeout=1)
            except serial.serialutil.SerialException:
                None
    

def main(args=None):
    rclpy.init()
    sr = cwt_npkphcth_reader()
    rclpy.spin(sr)

    
    sr.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()