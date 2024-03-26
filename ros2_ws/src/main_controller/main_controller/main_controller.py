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

        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.publisher_ = self.create_publisher(JoyID, 'joyid', 10)

        self.number_of_buttons = 11
        self.number_of_axes = 8

        #TODO It should get this number from input parameter
        self.number_of_drone_nodes = 2

        self.ID = 0
        self.selector_f = 0
        self.id_selector_counter = 1


    def listener_callback(self, msg):
        tx_msg = JoyID()

        # Press the controller's Back/Select and Start/Play buttons to change the mode to drone ID selector
        if(1 == msg.buttons[6]) and (1 == msg.buttons[7]):
            self.selector_f = 1

        if(self.selector_f and (1 == msg.buttons[7]) and (1 != msg.buttons[6])):
            self.selector_f = 0

        if(0 == self.selector_f):
            for i in range(0, self.number_of_buttons):           
                tx_msg.buttons[i] = msg.buttons[i]            
            
            for i in range(0, self.number_of_axes):
                tx_msg.axes[i] = msg.axes[i]      
        else:
            # Controller Cross up/down => change movement speed
            if((1 == int(msg.axes[7])) or (-1 == int(msg.axes[7]))):
                if(0 == (self.id_selector_counter % 5)):
                    self.id_selector_counter = 1

                    self.ID = self.change_id(int(msg.axes[7]), self.ID, self.number_of_drone_nodes)

                    
                else:
                    self.id_selector_counter += 1
        




        tx_msg.id = self.ID        
        self.publisher_.publish(tx_msg)

    def timer_callback(self):
        self.console_log()

    def console_log(self):
        os.system('clear')
        print("Last ID of the Drone Nodes: %d" % self.number_of_drone_nodes)
        print(" Chosen ID: %d" % self.ID)
        if(self.selector_f):
            print(" Mode: Drone Selecting")
        else:
            print(" Gatwaying Joy Messages")

    def change_id(self, axes_value, input_id, max_id = 1):
        if(1 == int(axes_value)):
            if(input_id < max_id):
                input_id += 1        

        elif (-1 == int(axes_value)):
            if(input_id > 0):
                input_id -= 1 
        
        return input_id



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