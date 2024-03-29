import time
import sys

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fungi_msgs.action import Countinng

class CountdownServer(Node):
    def __init__(self):
        # self.ID = args[0]        
        self.selected_action = Countinng

        super().__init__("countdown_server")
        
        self.declare_parameter('ID', rclpy.Parameter.Type.INTEGER)
        self.ID = self.get_parameter('ID')
        print("ID: %d" % self.ID.value)

        
        if(0 == self.ID.value):
            print("0")
        elif(1 == self.ID.value):
            print("1")
        else:
            print("Nem mukodik")
        
        



        self._action_server = ActionServer(
            self,
            self.selected_action,
            "countdown",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    # Callback function to run after acknowledging a goal from the client
    async def execute_callback(self, goal_handle):
        self.get_logger().info("Starting countdown…")

        feedback_msg = self.selected_action.Feedback()

        # Initiate the feedback message’s current_num as the action request’s starting_num
        feedback_msg.current_num = goal_handle.request.starting_num

        while feedback_msg.current_num > 0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return self.selected_action.Result()
            
	        # Decrement the feedback message’s current_num
            feedback_msg.current_num = feedback_msg.current_num - 1

           # Print log messages
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.current_num))
            goal_handle.publish_feedback(feedback_msg)

	        # Wait a second before counting down to the next number
            time.sleep(1)

        goal_handle.succeed()
        result = self.selected_action.Result()
        result.is_finished = True
        return result
    
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

def main(args=None):
    rclpy.init(args=None)
    countdown_server = CountdownServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(countdown_server, executor=executor)

    countdown_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])