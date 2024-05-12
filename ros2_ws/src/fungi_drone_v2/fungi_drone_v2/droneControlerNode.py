
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
from sensor_msgs.msg import Joy, Image

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
from guided_set_speed_yaw_modified import goto, condition_yaw, goto_position_target_global_int_mod, send_ned_velocity, set_vehicle_speed_guide
from guided_set_speed_yaw_modified import RC_Converter, send_global_velocity, arm_disarm, set_vehicle_speed, goto_position_target_global_int_mod_v2
from drone_kit import arm_and_takeoff


# from guided_set_speed_yaw import goto_position_target_global_int

# from mavproxy_fakegps import init

# import json
import positions as pos
from fungi_msgs.msg import JoyID
import image_saver
from cv_bridge import CvBridge, CvBridgeError

import calculations as calc




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
        self.debug_enable = True
        self.simulation_active = True
        self.console_on = True
        self.debug_pose = None
        self.debug_pose_flag = True
        self.debug_pose_status = 0
        self.debug_pose_object = None
        self.debug_pose_timer = None

        # if(self.debug_pose_flag):
        #     self.debug_pose_timer = self.create_timer(0.2, self.debug_pose_timer_callback)

        # BASE Frame
        self.base_frame = LocationGlobal(-35.3632621, 149.1650666, 584.0) #585.19


        self.timer_period = 0.2  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.timer_counter = 10
        self.seconds = [0, 0]

        self.scaller = 100
        self.vehicle_speed = 0.2
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

        # Image saver relevan variables
        self.image_save_enable = True
        self.image_saver_flag = False
        self.image_saver_timer_period = 0.2
        self.image_saver_counter = 0
        self.image_saver_timer = None
        self.bridge = CvBridge()

        if(self.image_save_enable):
            self.subscription = self.create_subscription(
                Image,
                '/camera/image',
                self.image_listener_callback,
                10)
        

        # Position variables
        self.positions = None #pos.Positions(init_from_file = False)
        self.home_pose = None

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

        self.home_pose = self.vehicle.location.global_frame

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
        

        self.gimbal_rotate(0)
         
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

        # TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST
        # self.image_saver = image_saver.ImageSaver(self.ID)
        # self.image_saver_timer = self.create_timer(self.image_saver_timer_period, self.image_saver_timer_callback)        
        # TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST

    def debug_pose_timer_callback(self):
        if(self.debug_pose_status == 1):
            self.debug_pose_object.save_a_debug_pose(self.debug_pose, self.vehicle.location.global_frame)
    
    def generate_execute_callback(self, selected_action):
        async def execute_callback(goal_handle):
            self.get_logger().info("Starting Mission")

            feedback_msg = self.selected_action.Feedback()

            # Initiate the feedback message’s current_num as the action request’s starting_num
            feedback_msg.current_num = goal_handle.request.start

            #TODO If the pose file is empty then reject the mission
            target_positions = pos.Positions(self.ID, self.base_frame, init_from_file = True)

    # LOGIC START
            # while feedback_msg.current_num > 0:

            # self.vehicle.mode = VehicleMode("GUIDED")
            arm_and_takeoff(self.vehicle, 3, "GUIDED")

            # Create an image saver object for the mission
            self.image_saver = image_saver.ImageSaver(self.ID, self.base_frame) 

            if(self.debug_pose_flag):
                self.debug_pose_object = pos.DebugPoseSaver(self.ID, self.base_frame)
                self.debug_pose_status = 1 
                self.debug_pose_timer = self.create_timer(0.2, self.debug_pose_timer_callback)
                  

            for ID in range(1,target_positions.get_pose_ID()):    #-1   

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')

                    if(self.image_save_enable):
                        if(None != self.image_saver_timer):
                            self.image_saver_timer.cancel()
                            self.image_saver_timer = None
                            
                        if(self.image_saver_counter > 0):
                            self.image_saver.save_poses_json()
                    
                    if(self.debug_pose_flag):
                        self.debug_pose = None

                    self.vehicle.mode = VehicleMode("LAND")
                    time.sleep(10)

                    return self.selected_action.Result()

                # If the pose allows to make images then the image saver timer starts
                if(self.image_save_enable):
                    if(1 == target_positions.get_imageSave_flag(ID)):
                        self.image_saver_timer = self.create_timer(self.image_saver_timer_period, self.image_saver_timer_callback)               

                pose = target_positions.get_pose(ID)
                new_pose = LocationGlobal(float(pose["lat"]), float(pose["lon"]), float(pose["alt"])) 
                self.gimbal_rotate(int(pose["orientation"]))

                if(self.debug_pose_flag):
                    self.debug_pose = new_pose



                # self.vehicle.simple_goto(new_pose, airspeed = 0.5) # --> Tökjól működik!!!!!
                # goto_position_target_global_int_mod(self.vehicle, new_pose)
                goto_position_target_global_int_mod_v2(self.vehicle, new_pose)

                # Increment the feedback message’s current_num
                feedback_msg.current_num +=  1

                # Print log messages
                # self.get_logger().info('Feedback: {0}'.format(feedback_msg.current_num))
                goal_handle.publish_feedback(feedback_msg)

                if(self.image_save_enable):
                # Turn off the Timer and set to the default value               
                    if(None != self.image_saver_timer):
                        self.image_saver_timer.cancel()
                        self.image_saver_timer = None
                
                # Wait a second before counting down to the next number
                time.sleep(1)
                print("self.vehicle.location.global_frame", self.vehicle.location.global_frame)
                print("lat_diff: ", calc.haver(self.base_frame.lat, self.base_frame.lon, self.vehicle.location.global_frame.lat, self.base_frame.lon))
                print("lon_diff: ", calc.haver(self.base_frame.lat, self.base_frame.lon, self.base_frame.lat, self.vehicle.location.global_frame.lon))
                print("alt diff: ", (self.vehicle.location.global_frame.alt - self.base_frame.alt))
            if(self.image_save_enable):
                # Mission end save the poses which has taken with the images as well
                self.image_saver.save_poses_json()
                self.image_saver_counter = 0
            time.sleep(2)
            # # Return to the base
            # for ID in range(target_positions.get_pose_ID()-2, -1, -1):
            #     if goal_handle.is_cancel_requested:
            #         goal_handle.canceled()
            #         self.get_logger().info('Goal canceled')
            #         return self.selected_action.Result()
                
            #     pose = target_positions.get_pose(ID)
            #     new_pose = LocationGlobal(float(pose["lat"]), float(pose["lon"]), float(pose["alt"])) #(pose["lat"], pose["lon"], pose["alt"]) 
            #     # self.vehicle.simple_goto(new_pose, airspeed = 0.5) # --> Tökjól működik!!!!!
            #     goto_position_target_global_int_mod_v2(self.vehicle, new_pose)

            #     # Increment the feedback message’s current_num
            #     feedback_msg.current_num +=  1

            #     # Print log messages
            #     # self.get_logger().info('Feedback: {0}'.format(feedback_msg.current_num))
            #     goal_handle.publish_feedback(feedback_msg)
            #     time.sleep(1)

                #Turn around
            # condition_yaw(self.vehicle, 180, True)
            # time.sleep(5)
                #Land

            if(self.debug_pose_flag):
                self.debug_pose_status = 0
                self.debug_pose = None
                self.debug_pose_object.save_poses_json()
                self.debug_pose_timer.cancel()
                

            self.vehicle.mode = VehicleMode("LAND")
            time.sleep(15)

            #TODO Disarmed and wait until vehicle is disarmed

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
                self.vehicle.gimbal.rotate(0, cam_yaw, 0) # self.cam_pitch, self.cam_roll
                print("cam_yaw", cam_yaw)
        # TODO Write here the necessary IO function which can handle the camera's servo state    


    def __del__(self):
        self.vehicle.close()
        print("vehicle disconnected")

    def timer_callback(self):
        if(self.console_on):
            self.console_log(self.vehicle)
            
        # if(self.debug_enable):
        #     self.dummy_local_var += 1 
        #     print(self.dummy_local_var)
        #     if(self.dummy_local_var == 20):            
        #         self.gimbal_rotate(-90)
                
        #     if(self.dummy_local_var == 40):            
        #         self.gimbal_rotate(90)
        #         self.dummy_local_var = 0
        
    # In real case it should take the image directly via cv2 API not from ROS to Image topic
    def image_saver_timer_callback(self):
        self.image_saver_flag = True
        # self.image_saver_counter += 1  #TODO Remove from the code, this is obsolete

    def image_listener_callback(self, msg):
        if(True == self.image_saver_flag):
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_saver.save_an_image(cv2_img, self.vehicle.location.global_frame)
            self.image_saver_flag = False

    def console_log(self, veichle):
        os.system('clear') 
        # print("\n\nTime between two cals: %f\n\n" % (self.seconds[1] - self.seconds[0]))
        print(" Drone ID: %d" % self.ID)
        print(" Current flight mode: %s" % self.vehicle.mode.name)
        print(" Chosen  flight mode: %s" % self.flight_modes[self.flight_mode_index])
        print(" Veichle armed: %d" % self.vehicle.armed)
        print(" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        print(" Is Armable?: %s" % self.vehicle.is_armable)
        print(" System status: %s" % self.vehicle.system_status.state)
        print(" Mission mode: %s" % self.mission_modes[self.mission_mode_index])   
        print(" Mission status: %s" % self.start_mission_status[self.start_mission_index])  
        # print(" Battery status: %s" % self.vehicle.battery.level)
        # print(" Battery Voltage: %s" % self.vehicle.battery.voltage)
        # print(" Battery Current: %s" % self.vehicle.battery.current)
        
        if("GUIDED" == self.vehicle.mode.name):
                    print(" Vehicle airspeed: %f" % self.vehicle_speed) #self.vehicle.airspeed)
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
                        arm_and_takeoff(self.vehicle, 3, "GUIDED")    
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
                            self.vehicle_speed = set_vehicle_speed_guide(self.vehicle_speed, msg.axes[7])
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
                            # TODO Check is the pose object exist or not
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
                                self.positions = pos.Positions(self.ID, self.base_frame, init_from_file = False) 
                                # Save Home Pose
                                self.positions.store_coordinates_dict(self.home_pose.lat,
                                                                      self.home_pose.lon,
                                                                      self.home_pose.alt)
                                self.saved_pose_indicator = 1
                                  
                            elif("APPEND_POSE" == self.mission_modes[self.mission_mode_index]): 
                                if(None == self.positions):
                                    self.positions = pos.Positions(self.ID, self.base_frame, init_from_file = True)                            
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
                            send_global_velocity(self.vehicle, (msg.axes[0] * self.vehicle_speed),
                                                 (msg.axes[1] * self.vehicle_speed), ((-0.1 * msg.axes[4]) * self.vehicle_speed ), 1)  
                               
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