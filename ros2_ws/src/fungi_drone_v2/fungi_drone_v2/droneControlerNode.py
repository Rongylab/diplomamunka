
#!/usr/bin/env python3
import sys
import os 
import time

# scripts relative path added 
sys.path.append(os.path.dirname(__file__))

# /home/drone/ardupilot/modules/mavlink/pymavlink/mavparm.py
# sys.path.append(os.path.dirname(0, "/home/drone/ardupilot/modules/mavlink/pymavlink/mavparm.py"))



import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy

# import pymavlink
# from mavparm import MAVParmDict
from pymavlink import mavparm
from pymavlink import mavutil

print("Start simulator (SITL)")
# import dronekit_sitl
# import time
# import math
# sitl = dronekit_sitl.start_default()
# connection_string = sitl.connection_string()

# Import DroneKit-Python
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
# import calculations as calc
from guided_set_speed_yaw_modified import goto, condition_yaw, goto_position_target_global_int_mod, send_ned_velocity
from guided_set_speed_yaw_modified import RC_Converter, send_global_velocity, arm_disarm, set_vehicle_speed, goto_position_target_global_int_mod_v2
from drone_kit import arm_and_takeoff


# from guided_set_speed_yaw import goto_position_target_global_int

# from mavproxy_fakegps import init

# import json
import positions as pos
from fungi_msgs.msg import JoyID




# import tf
# from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
# from mavros_msgs.msg import OverrideRCIn
# from mavros_msgs.msg import RCIn
# from mavros_msgs.srv import CommandBool
# from mavros_msgs.srv import SetMode
# from mavros_msgs.srv import CommandTOL
import math
# pi_2 = math.pi / 2.0

import time
import sys

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from fungi_msgs.action import Automission0





class droneControler(Node):    

    def __init__(self):
        super().__init__("droneControler")
        self.subscription = self.create_subscription(
            JoyID,
            'joyid',
            self.listener_callback,
            10)
        self.subscription

        # DEBUG Switch
        self.debug_enable = False
        self.simulation_active = True


        self.timer_period = 0.2  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.timer_counter = 10
        self.seconds = [0, 0]

        self.scaller = 100
        self.returm_value = 0

        self.flight_modes = ["GUIDED", "STABILIZE", "ALT_HOLD", "LAND"] #AIRMODE
        self.flight_mode_index = 0
        self.flight_mode_counter  = 1
        self.log_counter = 1
        
        self.rc_scaller = 50
        self.airspeed_counter = 1
        # [0.1; 1] m/s
        # TODO create a function which set this value
        self.guided_airspeed_scaller = 0.5

        self.save_pose_counter = 1
        self.save_pose_displayed_counter = 1
        self.saved_pose_indicator = 0
        self.file_writing_indicator = 0

        self.mission_modes = ["On_MISSION", "START_POSE_COLLECTION", "APPEND_POSE", "RETURN_HOME", "STOP_MISSION"]
        self.mission_mode_index = 1
        self.mission_mode_counter = 1
        self.start_mission_counter = 1
        self.start_mission_status = ["Idle", "Ongoing"]
        self.start_mission_index = 0

        self.positions = None #pos.Positions(init_from_file = False)

        # TODO add ERROR displays
        self.ERROR_message = None

        #TODO add ID from parameter
        self.ID = 0
                
        #TODO IP address change according to the input parameters
        print("Connecting to vehicle on: %s" % ("127.0.0.1:14550"))
        self.vehicle = connect('127.0.0.1:14550', wait_ready=True)
        # # Get some vehicle attributes (state)
        print("Get some vehicle attribute values:")
        print(" GPS: %s" % self.vehicle.gps_0)
        print(" Battery: %s" % self.vehicle.battery)
        print(" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        print(" Is Armable?: %s" % self.vehicle.is_armable)
        print(" System status: %s" % self.vehicle.system_status.state)
        print(" Mode: %s" % self.vehicle.mode.name )   # settable
        print("Global Location: %s" % self.vehicle.location.global_frame)
        print("Global Location (relative altitude): %s" % self.vehicle.location.global_relative_frame)

        self.vehicle.mode    = VehicleMode("GUIDED")
        self.vehicle.airspeed = 0.05
        self.vehicle.groundspeed = 0.05

        # cam_pitch and cam_roll can be ignored, it is necessary for the simulation purposes
        self.cam_pitch = 0
        self.cam_roll = 0
        
        self.dummy_local_var = 0


        # Action server settings

        # module_name = "module_1"
        # module_obj = __import__(module_name)

        self.selected_action = Automission0


        self._action_server = ActionServer(
            self,
            self.selected_action,
            f"missionsaction{self.ID}",
            execute_callback=self.generate_execute_callback(self.selected_action),
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
         
        # self.y = pos.Positions(init_from_file = True)
       
        # arm_and_takeoff(self.vehicle, 2)

        # for ID in range(self.y.get_pose_ID()):
        #     print(ID)
        #     pose = self.y.get_pose(ID)
        #     new_pose = LocationGlobal(pose["lat"], pose["lon"], pose["alt"])   

        #     print("Current Pose:")
        #     print(self.vehicle.location.global_frame) 
        #     print("New Pose:")
        #     print(new_pose)
        #     print("") 
        #     # vehicle.simple_goto(new_pose) # --> Tökjól működik!!!!!
        #     goto_position_target_global_int_mod(self.vehicle, new_pose)

        #     # goto_position_target_global_int(vehicle, new_pose)
        #     print("Reached Pose:")
        #     print(self.vehicle.location.global_frame)
        #     print("")

        #     time.sleep(5)
    
    def generate_execute_callback(self, selected_action):
        async def execute_callback(goal_handle):
            self.get_logger().info("Starting Mission")

            feedback_msg = self.selected_action.Feedback()

            # Initiate the feedback message’s current_num as the action request’s starting_num
            feedback_msg.current_num = goal_handle.request.start

            target_positions = pos.Positions(self.ID, init_from_file = True)

    # LOGIC START
            # while feedback_msg.current_num > 0:

            # self.vehicle.mode = VehicleMode("GUIDED")
            arm_and_takeoff(self.vehicle, 2, "GUIDED")  


            for ID in range(target_positions.get_pose_ID()):            
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    return self.selected_action.Result()
                
                pose = target_positions.get_pose(ID)
                new_pose = LocationGlobal(float(pose["lat"]), float(pose["lon"]), float(pose["alt"])) 
                # self.vehicle.simple_goto(new_pose, airspeed = 0.5) # --> Tökjól működik!!!!!
                # goto_position_target_global_int_mod(self.vehicle, new_pose)
                goto_position_target_global_int_mod_v2(self.vehicle, new_pose)

                # Increment the feedback message’s current_num
                feedback_msg.current_num +=  1

                # Print log messages
                # self.get_logger().info('Feedback: {0}'.format(feedback_msg.current_num))
                goal_handle.publish_feedback(feedback_msg)

                # Wait a second before counting down to the next number
                time.sleep(1)
            
            time.sleep(2)
            # Return to the base
            for ID in range(target_positions.get_pose_ID()-2, -1, -1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    return self.selected_action.Result()
                
                pose = target_positions.get_pose(ID)
                new_pose = LocationGlobal(float(pose["lat"]), float(pose["lon"]), float(pose["alt"])) #(pose["lat"], pose["lon"], pose["alt"]) 
                # self.vehicle.simple_goto(new_pose, airspeed = 0.5) # --> Tökjól működik!!!!!
                goto_position_target_global_int_mod_v2(self.vehicle, new_pose)

                # Increment the feedback message’s current_num
                feedback_msg.current_num +=  1

                # Print log messages
                # self.get_logger().info('Feedback: {0}'.format(feedback_msg.current_num))
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1)

                #Turn around
            # condition_yaw(self.vehicle, 180, True)
            # time.sleep(5)
                #Land
            self.vehicle.mode = VehicleMode("LAND")
            time.sleep(10)

    # LOGIC END
            # self.timer = self.create_timer(self.timer_period, self.timer_callback)
            goal_handle.succeed()
            result = self.selected_action.Result()
            result.is_finished = True
            return result
        return execute_callback
    
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel

        # self.timer.cancel()
        self.get_logger().info('Received goal request')
        self.mission_mode_index = 0
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


    def gimbal_rotate(self, cam_yaw = 0):
        if(self.debug_enable == True):
            if(self.simulation_active == True):
                self.vehicle.gimbal.rotate(self.cam_pitch, self.cam_roll, cam_yaw)
                print("cam_yaw", cam_yaw)
        # TODO Write here the necessary IO function which can handle the camera's servo state    


    def __del__(self):
        self.vehicle.close()
        print("vehicle disconnected")

    def timer_callback(self):
        
        self.console_log(self.vehicle)
        # self.dummy_local_var += 1 
        # print(self.dummy_local_var)
        # if(self.dummy_local_var == 20):            
        #     self.gimbal_rotate(0)
            
        # if(self.dummy_local_var == 40):            
        #     self.gimbal_rotate(90)
        #     self.dummy_local_var = 0
        

    def console_log(self, veichle):
        os.system('clear') 
        print("\n\nTime between two cals: %f\n\n" % (self.seconds[1] - self.seconds[0]))
        print(" Drone ID: %d" % self.ID)
        print(" Current flight mode: %s" % self.vehicle.mode.name)
        print(" Chosen  flight mode: %s" % self.flight_modes[self.flight_mode_index])
        print(" Veichle armed: %d" % self.vehicle.armed)
        print(" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        print(" Is Armable?: %s" % self.vehicle.is_armable)
        print(" System status: %s" % self.vehicle.system_status.state)
        print(" Mission mode: %s" % self.mission_modes[self.mission_mode_index])   
        print(" Mission status: %s" % self.start_mission_status[self.start_mission_index])  
        
        if("GUIDED" == self.vehicle.mode.name):
                    print(" Vehicle airspeed: %f" % self.vehicle.airspeed)
        elif("STABILIZE" == self.vehicle.mode.name):
                    print(" Vehicle RC_scaller: %d" % self.rc_scaller)   
        
        # After a pose saving the indicator displays until 5 sec
        if(1 == self.saved_pose_indicator):
            if(1 == self.file_writing_indicator):
                print(" Pose list was WRITEN into a file")  
            else:     
                print(" Pose SAVED to the list")  
            if(0 != (self.save_pose_displayed_counter % 26)):
                self.save_pose_displayed_counter += 1
            else:
                self.save_pose_displayed_counter = 1
                self.saved_pose_indicator = 0
                self.file_writing_indicator = 0      

      
    # def console_log(self, veichle, msg):
    #     os.system('clear') 
    #     print("current flight mode: %s" % self.vehicle.mode.name)
    #     print("chosen  flight mode: %s" % self.flight_modes[self.flight_mode_index])
    #     print("Veichle armed: %d" % self.vehicle.armed)
    #     print(" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
    #     print(" Is Armable?: %s" % self.vehicle.is_armable)
    #     print(" System status: %s" % self.vehicle.system_status.state)
    #     print(" Mode: %s" % self.vehicle.mode.name )   # settable
    #     print("\n\nTime between two cals: %f\n\n" % (self.seconds[1] - self.seconds[0]))
        
        

    def listener_callback(self, msg):
        # self.console_log(self.vehicle, msg)

        
        if(0 == (self.timer_counter % 10)):
            self.timer_counter = 10

            if(self.ID == msg.id): # msg.id self.ID == 0
                # if(0 == (self.log_counter % 10)):
                #     self.log_counter = 1                
                #     self.console_log(self.vehicle, msg)
                # else:
                #     self.log_counter += 1
            

                # Controller Y => armed/disarmed  
                if(1 == msg.buttons[3]):
                    if("GUIDED" == self.vehicle.mode.name):
                        arm_and_takeoff(self.vehicle, 2, "GUIDED")    
                    if("STABILIZE" == self.vehicle.mode.name):
                        arm_and_takeoff(self.vehicle, 0, "STABILIZE")
                    # arm_disarm(self.vehicle)
                    
                # Controller Cross Lef/Right => change mode 
                if((1 == int(msg.axes[6])) or (-1 == int(msg.axes[6]))):

                    if(0 == (self.flight_mode_counter % 5)):
                        self.flight_mode_counter = 1
                        self.seconds[0] = self.seconds[1]
                        self.seconds[1] = time.time()
                        # print("\n\nTime between two cals: %f\n\n" % (self.seconds[1] - self.seconds[0]))

                        if(1 == int(msg.axes[6])):
                            if(self.flight_mode_index == 0):
                                self.flight_mode_index =  len(self.flight_modes) - 1
                            else:
                                self.flight_mode_index -= 1

                        if(-1 == int(msg.axes[6])):
                            if(self.flight_mode_index == (len(self.flight_modes) - 1)):
                                self.flight_mode_index =  0
                            else:
                                self.flight_mode_index += 1
                    else:
                        self.flight_mode_counter += 1
                    
                # Controller RB accept the new flight mode 
                if(1 == msg.buttons[5]):   
                    self.vehicle.mode  = VehicleMode(self.flight_modes[self.flight_mode_index])         

                # Controller Cross up/down => change movement speed 
                if((1 == int(msg.axes[7])) or (-1 == int(msg.axes[7]))):
                    if(0 == (self.airspeed_counter % 5)):
                        self.airspeed_counter = 1

                        if("GUIDED" == self.vehicle.mode.name): 
                            set_vehicle_speed(self.vehicle, msg.axes[7])
                        elif("STABILIZE" == self.vehicle.mode.name):
                            self.rc_scaller = set_vehicle_speed(msg.axes[7], self.rc_scaller)
                    else:
                        self.airspeed_counter += 1

                # Controller RT => Save pose 
                if(("START_POSE_COLLECTION" == self.mission_modes[self.mission_mode_index]) or
                ("APPEND_POSE" == self.mission_modes[self.mission_mode_index])):      
                    if(-1 == int(msg.axes[5])): 
                        if(0 == (self.save_pose_counter % 10)):
                            self.save_pose_counter = 1
                            # TODO call the SAVE function
                            self.saved_pose_indicator = 1
                            self.positions.store_coordinates_dict(self.vehicle.location.global_frame.lat,
                                                                  self.vehicle.location.global_frame.lon,
                                                                  self.vehicle.location.global_frame.alt)
                        else:
                            self.save_pose_counter += 1 
                            
                # Controller Back/Select => Change mission mode
                if(1 == int(msg.buttons[6])): 
                    if(0 == (self.mission_mode_counter % 10)):
                        self.mission_mode_counter = 1

                        if(self.mission_mode_index == (len(self.mission_modes) - 1)):
                            self.mission_mode_index =  0
                        else:
                            self.mission_mode_index += 1                    
                    else:
                        self.mission_mode_counter += 1

                    # self.start_mission_counter 
                
                # Controller Start/Play => Start mission
                if(1 == int(msg.buttons[7])): 
                    if(0 == (self.start_mission_counter % 10)):
                        self.start_mission_counter = 1
                        
                        #Checks is there any ongoing mission or not
                        if(0 == self.start_mission_index):
                            self.start_mission_index = 1
                            if("START_POSE_COLLECTION" == self.mission_modes[self.mission_mode_index]):                            
                                # Create an empty position store object
                                self.positions = pos.Positions(self.ID, init_from_file = False)   
                            elif("APPEND_POSE" == self.mission_modes[self.mission_mode_index]): 
                                if(None == self.positions):
                                    self.positions = pos.Positions(self.ID, init_from_file = True)                            
                            # else:
                            #     self.start_mission_index = s0
                        else:
                            if(("START_POSE_COLLECTION" == self.mission_modes[self.mission_mode_index]) or 
                            ("APPEND_POSE" == self.mission_modes[self.mission_mode_index])):                            
                                self.positions.write_poses_to_file()
                                self.file_writing_indicator = 1  
                                self.saved_pose_indicator = 1 
                            if("STOP_MISSION" == self.mission_modes[self.mission_mode_index]):
                                self.start_mission_index = 0
                                   
                else:
                    self.start_mission_counter += 1

            


                # Controller LB => Enable button
                if(1 == msg.buttons[4]):

                    if("GUIDED" == self.vehicle.mode.name):
                        if(0 == (msg.axes[0])) and (0 == (msg.axes[1])) and (0 == (msg.axes[4])):   
                            # send yaw intruction from the controller
                            condition_yaw(self.vehicle, (round(msg.axes[3] * self.scaller , 6)), True) # math.degrees(self.vehicle.attitude.yaw) +     
                        else:
                            # Send velocity from the controller              
                            # send_ned_velocity(self.vehicle, msg.axes[0], msg.axes[1], (-1 * msg.axes[4]), 1) 
                            send_global_velocity(self.vehicle, msg.axes[0], msg.axes[1], (-1 * msg.axes[4]), 1)     
                    elif("STABILIZE" == self.vehicle.mode.name):
                        # # Channel 1: Roll <-- msg.axes[3]                    
                        # # Channel 2: Pitch <-- msg.axes[1]                    
                        # # Channel 3: Throttle <-- msg.axes[4]                   
                        # # Channel 4: Yaw <-- msg.axes[0]
                        self.vehicle.channels.overrides = {'1': RC_Converter(msg.axes[3], self.rc_scaller, -1), '2': RC_Converter(msg.axes[1], self.rc_scaller, -1),
                                                        '3': RC_Converter(msg.axes[4], self.rc_scaller),     '4': RC_Converter(msg.axes[0], self.rc_scaller, -1)}
        else:
            self.timer_counter += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = droneControler()
    executor = MultiThreadedExecutor()

    rclpy.spin(minimal_subscriber, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()