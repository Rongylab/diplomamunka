print("Start simulator (SITL)")
import dronekit_sitl
import time
import math
# sitl = dronekit_sitl.start_default()
# connection_string = sitl.connection_string()

# Import DroneKit-Python
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import calculations as calc
from guided_set_speed_yaw_modified import goto, condition_yaw, goto_position_target_global_int_mod
# from guided_set_speed_yaw import goto_position_target_global_int

from mavproxy_fakegps import init
from pymavlink import mavutil

import json
import positions as pos


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print ("Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.70:
            print("Reached target altitude")
            break
        time.sleep(1)

def simple_goto(vehicle, x, y, z):
    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon
    alt = vehicle.location.global_frame.alt

    lat_dest = lat + y * 9.013373
    long_dest = lon + x * (8.98315/(math.cos(lat/1000000)))

    a_location = LocationGlobal(lat_dest, long_dest, alt + z)
    vehicle.simple_goto(a_location)
    
    while True:
        print ("Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if (vehicle.location.global_relative_frame.lat >= (a_location.lat) * 0.70) & (vehicle.location.global_relative_frame.lon >= (a_location.lon)*0.70) & (vehicle.location.global_relative_frame.alt >= (a_location.alt+z)*0.70):
            print("Reached target altitude")
            break
        time.sleep(1)



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

    vehicle.send_mavlink(msg)
    
    # # send command to vehicle on 1 Hz cycle
    # for x in range(0,duration):
    #     vehicle.send_mavlink(msg)
    #     time.sleep(1)   


def goto_original(dNorth, dEast, vehicle):

    remainingDistance_cnt = 0
    remainingDistance_prev = 0
    simple_pose_control_f = False

    gotoFunction=vehicle.simple_goto
    currentLocation=vehicle.location.global_relative_frame
    targetLocation=get_location_metres(currentLocation, dNorth, dEast)
    targetDistance=get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)


    print("")
    print("")
    print("dNorth:", dNorth)
    print("dEast", dEast)

    # send_global_velocity(self.vehicle, msg.axes[0], msg.axes[1], (-1 * msg.axes[4]), 1)     
    

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        remainingDistance=get_distance_metres(vehicle.location.global_frame, targetLocation)


        print("")
        if(simple_pose_control_f == False):
            print("remainingDistance_cnt:", remainingDistance_cnt)
        print("vehicle.location.global_frame", vehicle.location.global_frame)
        print("targetLocation", targetLocation)        

        print("Distance to target: ", remainingDistance)
        # print("targetDistance: ", targetDistance)
        # print("targetDistance * 0.33 ", targetDistance * 0.33 )


        if(remainingDistance <= (targetDistance*0.1)): #Just below target, in case of undershoot.
            print("Reached target")
            if(simple_pose_control_f):
                send_global_velocity(vehicle, 0, 0, 0, 1)
            break

        if(abs(remainingDistance - remainingDistance_prev) > 0.3):
            remainingDistance_prev = remainingDistance
            remainingDistance_cnt = 0

        elif(remainingDistance_cnt < 20):
            remainingDistance_cnt += 1
        else:
            simple_pose_control_f = True
        
        if(simple_pose_control_f):
            
            print("Mode simple pose control")
            pose2 = targetLocation
            pose1 = vehicle.location.global_frame

            distance_lat, distance_lon = calc.haversine2(pose1.lat, pose1.lon, pose2.lat, pose2.lon)

            print("distance_lat", distance_lat)
            print("distance_lon", distance_lon)

            if(distance_lat > 0):
                lat_x_vel = distance_lat if distance_lat < 0.2 else 0.2
            else:
                lat_x_vel = distance_lat if abs(distance_lat) < 0.2 else -0.2

            if(distance_lon > 0):
                lon_y_vel = distance_lon if distance_lon < 0.2 else 0.2
            else:
                lon_y_vel = distance_lon if abs(distance_lon) < 0.2 else -0.2
            

            print("lat_x_vel", lat_x_vel)
            print("lon_y_vel", lon_y_vel)

            send_global_velocity(vehicle, lat_x_vel, lon_y_vel, 0, 1)  

        # distance_lat, distance_lon = haversine2(pose1["lat"], pose1["lon"], pose2["lat"], pose2["lon"])
        # if remaining distance is the same at least 20 times then 
        time.sleep(0.1)
        



# TEST Begin


# Connect to the Vehicle.guided_set_speed_yaw
print("Connecting to vehicle on: %s" % ("127.0.0.1:14550"))
vehicle = connect('127.0.0.1:14550', wait_ready=True)
# print("Connecting to vehicle on: %s" % (connection_string,))
# vehicle = connect(connection_string, wait_ready=True)

# Get some vehicle attributes (state)
print("Get some vehicle attribute values:")
print(" GPS: %s" % vehicle.gps_0)
print(" Battery: %s" % vehicle.battery)
print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
print(" Is Armable?: %s" % vehicle.is_armable)
print(" System status: %s" % vehicle.system_status.state)
print(" Mode: %s" % vehicle.mode.name )   # settable
print("Global Location: %s" % vehicle.location.global_frame)
print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)




# Arm the drone and do a takeoff
arm_and_takeoff(2)
time.sleep(5)
print("Global Location: %s" % vehicle.location.global_frame)
time.sleep(2)

# # vehicle.airspeed = 0.05 # [m/s]

# condition_yaw(vehicle, 270, True)
# time.sleep(5)


# lat = vehicle.location.global_frame.lat
# lon = vehicle.location.global_frame.lon
# alt = vehicle.location.global_frame.alt
# print("lat %s", lat)

# print("\n\nMission Start")
# print("Global Location before mission: %s" % vehicle.location.global_frame)
# calc.print_DMS(vehicle)
# goto_position_target_global_int()

# y = pos.Positions(init_from_file = True)

# for ID in range(y.get_pose_ID()):
#     print(ID)
#     pose = y.get_pose(ID)
#     new_pose = LocationGlobal(pose["lat"], pose["lon"], pose["alt"])   

#     print("Current Pose:")
#     print(vehicle.location.global_frame) 
#     print("New Pose:")
#     print(new_pose)
#     print("") 
#     # vehicle.simple_goto(new_pose) # --> Tökjól működik!!!!!
#     goto_position_target_global_int_mod(vehicle, new_pose)
#     # goto_position_target_global_int(vehicle, new_pose)
#     print("Reached Pose:")
#     print(vehicle.location.global_frame)
#     print("")

#     time.sleep(5)

print("vehicle.home_location %s" % vehicle.home_location)
print("vehicle.location.global_frame %s" % vehicle.location.global_frame)

vehicle.home_location = vehicle.location.global_frame 
# print("vehicle.home_location %s" % vehicle.home_location)

print("")
goto_original(1, 0, vehicle)
# print("vehicle.location.global_frame %s" % vehicle.location.global_frame)
time.sleep(5)

goto_original(0.5, 0, vehicle)
# print("vehicle.location.global_frame %s" % vehicle.location.global_frame)
time.sleep(5)

goto_original(0.2, 0, vehicle)
# print("vehicle.location.global_frame %s" % vehicle.location.global_frame)
time.sleep(5)




# goto(vehicle, -5, 0)
# time.sleep(5)
# y.store_coordinates_dict(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)

# goto(vehicle, 0, 5)
# time.sleep(5)
# y.store_coordinates_dict(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)

# goto(vehicle, 5, 0)
# time.sleep(5)
# y.store_coordinates_dict(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)

# goto(vehicle, 0, -5)
# time.sleep(5)
# y.store_coordinates_dict(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)

# y.write_poses_to_file()

# print("Global Location after mission: %s" % vehicle.location.global_frame)
# calc.print_DMS(vehicle)

# print("Difference between start and destination points: %f" % calc.diff_in_meter(calc.DD2DMS(lat)[3], calc.DD2DMS(vehicle.location.global_frame.lat)[3], 0)) # VALAMI BAJA vana a selectorral!!!


# lat = vehicle.location.global_frame.lat
# lon = vehicle.location.global_frame.lon
# alt = vehicle.location.global_frame.alt
# print("global_frame %s", vehicle.location.global_frame)

# x = {
#     "pose_ID" : "0",
#     "lat": lat,
#     "lon": lon,
#     "alt": alt
# }

# def store_coordinates_dict(pose_ID, lat, lon, alt, orientation = 0):
#     dict = {
#     "pose_ID" : pose_ID,
#     "lat": lat,
#     "lon": lon,
#     "alt": alt,
#     "orientation": orientation
#     }
#     return dict


# y = []
# z = store_coordinates_dict(0, lat, lon, alt)
# y.append(json.dumps(z))
# y.append(json.dumps(z))
# y.append(json.dumps(z))

# y = pos.Positions()

# y.store_coordinates_dict(lat, lon, alt)
# y.store_coordinates_dict(lat, lon, alt)

# y.write_poses_to_file()

# print(y.get_positions())


# print("\n\nMission Start")
# print("Global Location before mission: %s" % vehicle.location.global_frame)
# calc.print_DMS(vehicle)

# goto(vehicle, 5, 0)

# print("Global Location after mission: %s" % vehicle.location.global_frame)
# calc.print_DMS(vehicle)

# print("Difference between start and destination points: %.3fm" % calc.diff_in_meter(calc.DD2DMS(lat)[3], calc.DD2DMS(vehicle.location.global_frame.lat)[3], 0))


# lat = vehicle.location.global_frame.lat
# lon = vehicle.location.global_frame.lon
# alt = vehicle.location.global_frame.alt
# print("lat %s", lat)

# print("\n\nMission Start")
# print("Global Location before mission: %s" % vehicle.location.global_frame)
# calc.print_DMS(vehicle)

# goto(vehicle, -5, 0)

# print("Global Location after mission: %s" % vehicle.location.global_frame)
# calc.print_DMS(vehicle)

# print("Difference between start and destination points: %.3fm" % calc.diff_in_meter(calc.DD2DMS(lat)[3], calc.DD2DMS(vehicle.location.global_frame.lat)[3], 0))


# a_location = LocationGlobal(-34.364114, 149.166022, 30) 
# a_location = LocationGlobal(lat, lon, alt + 8)
# vehicle.simple_goto(a_location)
# while True:
#         print ("Altitude: ", vehicle.location.global_relative_frame.alt)
#         #Break and return from function just below target altitude.
#         if vehicle.location.global_relative_frame.alt>=a_location.alt*0.70:
#             print("Reached target altitude")
#             break
#         time.sleep(1)

# a_location = LocationGlobalRelative(0, 0, 10)
# vehicle.simple_goto(a_location)
# while True:
#         print ("Altitude: ", vehicle.location.global_relative_frame.alt)
#         #Break and return from function just below target altitude.
#         if vehicle.location.global_relative_frame.alt>=a_location.lon*0.70:
#             print("Reached target altitude")
#             break
#         time.sleep(1)

# simple_goto(vehicle, 0, 0, 2)
# simple_goto(vehicle, 5, 0, 0)
# simple_goto(vehicle, 0, 5, 5)

# lat_DMS = calc.DD2DMS(lat)
# lat_dest_DMS = lat_DMS[3] + (calc.METER_CONST_PER_SEC * 5)

# arm_and_takeoff(5)
# time.sleep(3)
# print("Set default/target airspeed to 3")
# # vehicle.airspeed = 3

# print("Going towards first point for 30 seconds ...")
# point1 = LocationGlobalRelative(calc.DMS2DD(lat_DMS[0],lat_DMS[1],lat_DMS[2], lat_dest_DMS), 149.1652374, 5)
# vehicle.simple_goto(point1)

# print("Desired point: %s", point1)

# while True:
#     print ("Pose: ", vehicle.location.global_relative_frame)
#     #Break and return from function just below target altitude.
#     if(vehicle.location.global_relative_frame.lat >= (point1.lat) * 0.70) & (vehicle.location.global_relative_frame.lon >= (point1.lon)*0.70) & (vehicle.location.global_relative_frame.alt >= (point1.alt)*0.70):
#         print("Reached target pose")
#         break
#     time.sleep(1)

# lat_DMS = calc.DD2DMS(lat)
# lat_dest_DMS = lat_DMS[3] - (calc.METER_CONST_PER_SEC * 5)

# print("Going towards first point for 30 seconds ...")
# point1 = LocationGlobalRelative(calc.DMS2DD(lat_DMS[0],lat_DMS[1],lat_DMS[2], lat_dest_DMS), 149.1652374, 5)
# vehicle.simple_goto(point1)
# time.sleep(5)

# sleep so we can see the change in map
# time.sleep(30)

# print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
# point2 = LocationGlobalRelative(-35.363244, 149.1652374, 5)
# vehicle.simple_goto(point2, groundspeed=10)

# sleep so we can see the change in map
# time.sleep(30)

# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
# sitl.stop()
print("Completed")

# TEST End