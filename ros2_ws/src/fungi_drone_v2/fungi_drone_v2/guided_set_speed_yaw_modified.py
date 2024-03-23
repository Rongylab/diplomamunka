#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
guided_set_speed_yaw.py: (Copter Only)

This example shows how to move/direct Copter and send commands in GUIDED mode using DroneKit Python.

Example documentation: http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
"""
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

from calculations import get_location_metres_mod, get_distance_metres_mod


"""
Convenience functions for sending immediate/guided mode commands to control the Copter.

The set of commands demonstrated here include:
* MAV_CMD_CONDITION_YAW - set direction of the front of the Copter (latitude, longitude)
* MAV_CMD_DO_SET_ROI - set direction where the camera gimbal is aimed (latitude, longitude, altitude)
* MAV_CMD_DO_CHANGE_SPEED - set target speed in metres/second.


The full set of available commands are listed here:
http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/
"""

def condition_yaw(vehicle, heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle

    spining_direction = 0

    if heading > 0:
        spining_direction = -1   # counter clockwise
    else:
        spining_direction = 1    # clockwise

    heading = abs(heading)


    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        spining_direction,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def set_roi(vehicle, location):
    """
    Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
    specified region of interest (LocationGlobal).
    The vehicle may also turn to face the ROI.

    For more information see: 
    http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
    """
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)



"""
Functions to make it easy to convert between the different frames-of-reference. In particular these
make it easy to navigate in terms of "metres from the current position" when using commands that take 
absolute positions in decimal degrees.

The methods are approximations only, and may be less accurate over longer distances, and when close 
to the Earth's poles.

Specifically, it provides:
* get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.
* get_distance_metres - Get the distance between two LocationGlobal objects in metres
* get_bearing - Get the bearing in degrees to a LocationGlobal
"""

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """	
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing



"""
Functions to move the vehicle to a specified position (as opposed to controlling movement by setting velocity components).

The methods include:
* goto_position_target_global_int - Sets position using SET_POSITION_TARGET_GLOBAL_INT command in 
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT frame
* goto_position_target_local_ned - Sets position using SET_POSITION_TARGET_LOCAL_NED command in 
    MAV_FRAME_BODY_NED frame
* goto - A convenience function that can use Vehicle.simple_goto (default) or 
    goto_position_target_global_int to travel to a specific position in metres 
    North and East from the current location. 
    This method reports distance to the destination.
"""

def goto_position_target_global_int_mod(vehicle, aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """

    currentLocation = vehicle.location.global_relative_frame    
    targetDistance = get_distance_metres_mod(currentLocation, aLocation)

    # msg = vehicle.message_factory.set_position_target_global_int_encode(
    #     0,       # time_boot_ms (not used)
    #     0, 0,    # target system, target component
    #     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
    #     0b0000111111111000, # type_mask (only speeds enabled)
    #     aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
    #     aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
    #     aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
    #     0, # X velocity in NED frame in m/s
    #     0, # Y velocity in NED frame in m/s
    #     0, # Z velocity in NED frame in m/s
    #     0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
    #     0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # # send command to vehicle
    # vehicle.send_mavlink(msg)

    vehicle.simple_goto(aLocation)
    
    while (vehicle.mode.name == "GUIDED"): #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance = get_distance_metres_mod(vehicle.location.global_relative_frame, aLocation)
        print("Distance to target: ", remainingDistance)
        if(remainingDistance <= targetDistance*0.10): #Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(1)



def goto_position_target_local_ned(vehicle, north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see: 
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)



def goto(vehicle, dNorth, dEast):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres_mod(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres_mod(currentLocation, targetLocation)
    vehicle.simple_goto(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while (vehicle.mode.name == "GUIDED"): #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance = get_distance_metres_mod(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if(remainingDistance <= targetDistance*0.10): #Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(1)



"""
Functions that move the vehicle by specifying the velocity components in each direction.
The two functions use different MAVLink commands. The main difference is
that depending on the frame used, the NED velocity can be relative to the vehicle
orientation.

The methods include:
* send_ned_velocity - Sets velocity components using SET_POSITION_TARGET_LOCAL_NED command
* send_global_velocity - Sets velocity components using SET_POSITION_TARGET_GLOBAL_INT command
"""

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
    
    


def send_global_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.

    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)    

# #TODO!!!!!!!
# def degree_check(input_degree, modifier):
#     if(input_degree + modifier > 360):
#         modifier


def RC_Converter(input_data, scaller = 400, invert = 1):
    
    input_data *= invert
    return_value = 1500
    #[-1.1]
    if(input_data > 0):
        return_value += input_data * scaller #400
    elif(input_data < 0):
        return_value += input_data * scaller #400       

    return int(return_value)

"""
Example of how to use RC_CHANNEL_OVERRIDE messages to force input channels
in Ardupilot. These effectively replace the input channels (from joystick
or radio), NOT the output channels going to thrusters and servos.
"""

# # Import mavutil
# from pymavlink import mavutil

# # Create the connection
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# # Wait a heartbeat before sending commands
# master.wait_heartbeat()

# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(master, channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.


# # Set some roll
# set_rc_channel_pwm(2, 1600)

# # Set some yaw
# set_rc_channel_pwm(4, 1600)

# # The camera pwm value sets the servo speed of a sweep from the current angle to
# #  the min/max camera angle. It does not set the servo position.
# # Set camera tilt to 45º (max) with full speed
# set_rc_channel_pwm(8, 1900)

# # Set channel 12 to 1500us
# # This can be used to control a device connected to a servo output by setting the
# # SERVO[N]_Function to RCIN12 (Where N is one of the PWM outputs)
# set_rc_channel_pwm(12, 1500)
    
def Arm_copter(master):
    # Arm
    # master.arducopter_arm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')

def Disarm_copter(master):
    # Disarm
    # master.arducopter_disarm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

    # wait until disarming confirmed
    master.motors_disarmed_wait()


# TODO: Remove this function, because doesn't work properly
def change_flight_mode(master, mode = 'STABILIZE'):
    return_value = 0
    attempt_counter = 10
    timeout_counter = 1000000

    # Check if mode is available
    if mode not in master.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(master.mode_mapping().keys()))
        #sys.exit(1)
        return_value = 1

    if(0 == return_value):
        return_value = 1
        print("Change Flight mode to: %s" % mode)
        # Get mode ID
        mode_id = master.mode_mapping()[mode]
        # Set new mode
        # master.mav.command_long_send(
        #    master.target_system, master.target_component,
        #    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        #    0, mode_id, 0, 0, 0, 0, 0) or:
        # master.set_mode(mode_id) or:

        while(attempt_counter and return_value):
            timeout_counter = 1000
            print("attempt_counter: %d" % attempt_counter)
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id)

            print("Watining for aknowlede")
            while timeout_counter:
                # Wait for ACK command
                # Would be good to add mechanism to avoid endlessly blocking
                # if the autopilot sends a NACK or never receives the message
                ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
                ack_msg = ack_msg.to_dict()

                # Continue waiting if the acknowledged command is not `set_mode`
                if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE: 
                    timeout_counter -= 1
                    print("timeout_counter %d" % timeout_counter)           
                    continue
            

                # Print the ACK result !
                print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
                return_value = 0
                break

            attempt_counter -= 1
            time.sleep(1)

        if(0 >= attempt_counter):
            return_value = 0
        else:
            return_value = 1

    return return_value

def arm_disarm(vehicle):
    
    if(True == vehicle.armed):
        vehicle.armed   = False        
        while  vehicle.armed:
            print(" Waiting for disarming...")
            time.sleep(1)
        print("disarmed!") 

    else:
        vehicle.armed   = True
        # Confirm vehicle armed before attempting to take off
        while not vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)
        print("armed!")

""" Upper limit 10  [m/s]
    Lower limit 0.1 [m/s] """
def set_vehicle_speed(vehicle, axes_value):
    if(1 == int(axes_value)):
        if(vehicle.airspeed < 10):
            vehicle.airspeed += 0.1         

    elif (-1 == int(axes_value)):
        if(vehicle.airspeed > 0.1):
            vehicle.airspeed -= 0.1 

""" Upper limit 400  
    Lower limit 10  """
def set_vehicle_speed(axes_value, scaller = 400):
    if(1 == int(axes_value)):
        if(scaller < 400):
            scaller += 10        

    elif (-1 == int(axes_value)):
        if(scaller > 10):
            scaller -= 10 
    
    return scaller
    
