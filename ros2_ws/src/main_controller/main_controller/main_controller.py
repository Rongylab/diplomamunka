#!/usr/bin/env python3
import sys
import os 
import time

# scripts relative path added 
sys.path.append(os.path.dirname(__file__))

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from fungi_msgs.msg import JoyID



class mainController(Node):    

    def __init__(self):
        super().__init__("mainController")
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

        self.publisher_ = self.create_publisher(JoyID, 'joyid', 10)

        self.number_of_buttons = 11
        self.number_of_axes = 8


    def listener_callback(self, msg):
        tx_msg = JoyID()

        for i in range(0, self.number_of_buttons):           
            tx_msg.buttons[i] = msg.buttons[i]            
        
        for i in range(0, self.number_of_axes):
            tx_msg.axes[i] = msg.axes[i]       
        
        tx_msg.id = 27        
        self.publisher_.publish(tx_msg)




def main(args=None):
    rclpy.init(args=args)

    maincontroller_pubsub = mainController()

    rclpy.spin(maincontroller_pubsub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    maincontroller_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()