
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
from guided_set_speed_yaw_modified import RC_Converter, send_global_velocity, arm_disarm, set_vehicle_speed
from drone_kit import arm_and_takeoff


# from guided_set_speed_yaw import goto_position_target_global_int

# from mavproxy_fakegps import init

# import json
import positions as pos




# import tf
# from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
# from mavros_msgs.msg import OverrideRCIn
# from mavros_msgs.msg import RCIn
# from mavros_msgs.srv import CommandBool
# from mavros_msgs.srv import SetMode
# from mavros_msgs.srv import CommandTOL
import math

# pi_2 = math.pi / 2.0





class droneControler(Node):    

    def __init__(self):
        super().__init__("droneControler")
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

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
        self.guided_airspeed_scaller = 0.5

        self.save_pose_counter = 1
        self.save_pose_displayed_counter = 1
        self.saved_pose_indicator = 0
        self.file_writing_indicator = 0

        self.mission_modes = ["START_MISSION", "START_POSE_COLLECTION", "APPEND_POSE", "RETURN_HOME", "STOP_MISSION"]
        self.mission_mode_index = 1
        self.mission_mode_counter = 1
        self.start_mission_counter = 1
        self.start_mission_status = ["Idle", "Ongoing"]
        self.start_mission_index = 0

        self.positions = None #pos.Positions(init_from_file = False)

        

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
        self.vehicle.airspeed = 0.5        
        
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
        
    def __del__(self):
        self.vehicle.close()
        print("vehicle disconnected")

    def timer_callback(self):
        self.console_log(self.vehicle)

    def console_log(self, veichle):
        os.system('clear') 
        print("\n\nTime between two cals: %f\n\n" % (self.seconds[1] - self.seconds[0]))
        print(" Drone ID: %s" % "N.A.")
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
                            self.positions = pos.Positions(init_from_file = False)                                             
                        # else:
                        #     self.start_mission_index = 0
                    else:
                        if("START_POSE_COLLECTION" == self.mission_modes[self.mission_mode_index]):
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
                else:
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

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()