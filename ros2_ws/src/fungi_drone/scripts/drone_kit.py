print("Start simulator (SITL)")
import dronekit_sitl
import time
import math
# sitl = dronekit_sitl.start_default()
# connection_string = sitl.connection_string()

# Import DroneKit-Python
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import calculations as calc
from guided_set_speed_yaw_modified import goto, condition_yaw

from mavproxy_fakegps import init


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
# arm_and_takeoff(2)

# condition_yaw(vehicle, 270, True)
# time.sleep(5)


# lat = vehicle.location.global_frame.lat
# lon = vehicle.location.global_frame.lon
# alt = vehicle.location.global_frame.alt
# print("lat %s", lat)

# print("\n\nMission Start")
# print("Global Location before mission: %s" % vehicle.location.global_frame)
# calc.print_DMS(vehicle)

# goto(vehicle, -5, 0)
# time.sleep(5)
# goto(vehicle, 0, 5)
# time.sleep(5)
# goto(vehicle, 5, 0)
# time.sleep(5)
# goto(vehicle, 0, -5)
# time.sleep(5)

# print("Global Location after mission: %s" % vehicle.location.global_frame)
# calc.print_DMS(vehicle)

# print("Difference between start and destination points: %f" % calc.diff_in_meter(calc.DD2DMS(lat)[3], calc.DD2DMS(vehicle.location.global_frame.lat)[3], 0)) # VALAMI BAJA vana a selectorral!!!


lat = vehicle.location.global_frame.lat
lon = vehicle.location.global_frame.lon
alt = vehicle.location.global_frame.alt
print("lat %s", lat)

print("\n\nMission Start")
print("Global Location before mission: %s" % vehicle.location.global_frame)
calc.print_DMS(vehicle)

goto(vehicle, 5, 0)

print("Global Location after mission: %s" % vehicle.location.global_frame)
calc.print_DMS(vehicle)

print("Difference between start and destination points: %.3fm" % calc.diff_in_meter(calc.DD2DMS(lat)[3], calc.DD2DMS(vehicle.location.global_frame.lat)[3], 0))


lat = vehicle.location.global_frame.lat
lon = vehicle.location.global_frame.lon
alt = vehicle.location.global_frame.alt
print("lat %s", lat)

print("\n\nMission Start")
print("Global Location before mission: %s" % vehicle.location.global_frame)
calc.print_DMS(vehicle)

goto(vehicle, -5, 0)

print("Global Location after mission: %s" % vehicle.location.global_frame)
calc.print_DMS(vehicle)

print("Difference between start and destination points: %.3fm" % calc.diff_in_meter(calc.DD2DMS(lat)[3], calc.DD2DMS(vehicle.location.global_frame.lat)[3], 0))


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