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

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from rclpy.parameter import Parameter

from fungi_msgs.action import Automission0



class mainController(Node):    

    def __init__(self):
        super().__init__("mainController")
        
        self.AUTOMAT_MODE = True
        self.ID = 0
        self.selected_action = Automission0

        # Create joy topic subscription
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

        # Create Action client 
        self._action_client = ActionClient(self, Automission0, f"missionsaction{self.ID}")

        # Create Timers
        self.timer_period = 0.2  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Start mission timer
        self.star_misson_timer = self.create_timer(10, self.star_misson_timer_callback)
        
        if(self.AUTOMAT_MODE == False):
            self.star_misson_timer.cancel()

        # Create Pupblisher
        self.publisher_ = self.create_publisher(JoyID, 'joyid', 10)

        self.number_of_buttons = 11
        self.number_of_axes = 8

        #TODO It should get this number from input parameter
        self.number_of_drone_nodes = 1

        
        self.selector_f = 0
        self.id_selector_counter = 1
        self.misson = 0


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
            # A (CROSS) 
            if((1 == msg.buttons[0]) and (not self.misson)):
                self.misson = 1
                self.send_goal(True)




        tx_msg.id = self.ID        
        self.publisher_.publish(tx_msg)

    def timer_callback(self):
        self.console_log()

    def star_misson_timer_callback(self):
        self.send_goal(True)
        self.star_misson_timer.cancel()

    def console_log(self):
        os.system('clear')
        print(" Number of the Drone Nodes: %d" % self.number_of_drone_nodes)
        print(" Chosen ID: %d" % self.ID)
        if(self.selector_f):
            print(" Mode: Drone Selecting")
        else:
            print(" Mode: Gatwaying Joy Messages")

    # TODO max_id use parameter
    def change_id(self, axes_value, input_id, max_id = 1):
        if(1 == int(axes_value)):
            if(input_id < max_id):
                input_id += 1        

        elif (-1 == int(axes_value)):
            if(input_id > 0):
                input_id -= 1 
        
        return input_id

    # Action Client defs:
    # Waits for server to be available, then sends goal
    def send_goal(self, start):
        self.timer.cancel()
        print("Waiting for server")
        goal_msg = self.selected_action.Goal()
        goal_msg.start = start
        self._action_client.wait_for_server()

        print("Sending request")
        # Returns future to goal handle; client runs feedback_callback after sending the goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # Register a callback for when future is complete (i.e. server accepts or rejects goal request)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        print("Request Sent")

    # Run when client sends goal
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received Pose ID: {0}'.format(feedback.current_num))

    def goal_response_callback(self, future):
        # Get handle for the goal we just sent
        self.goal_handle = future.result()

        # Return early if goal is rejected
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Start a 2 second timer
        # self._timer = self.create_timer(2.0, self.timer_callback)

        # Use goal handle to request the result
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        # Log result and shut down ROS 2 cleanly
        self.get_logger().info('Result: {0}'.format(result.is_finished))
        self.misson = 0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.star_misson_timer = self.create_timer(10, self.star_misson_timer_callback)
        # rclpy.shutdown()
    
    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        # rclpy.shutdown()

    # def timer_callback(self):
    #     # self.get_logger().info('Canceling goal')
    #     # # Cancel the goal
    #     # future = self.goal_handle.cancel_goal_async()
    #     # future.add_done_callback(self.cancel_done)
    #     # print("done future")

    #     # Cancel the timer
    #     self._timer.cancel()

def main(args=None):
    rclpy.init(args=args)

    maincontroller_pubsub = mainController()
    executor = MultiThreadedExecutor()

    rclpy.spin(maincontroller_pubsub, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    maincontroller_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()