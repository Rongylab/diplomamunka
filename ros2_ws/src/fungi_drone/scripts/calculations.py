
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math


METER_CONST_PER_SEC_LAT = 32.435 * 10**(-3) # 1 [meter] equals to  32.435*10^-3 [sec]
CMETER_CONST_PER_SEC_LAT = 324.35 * 10**(-6) # 1 [cmeter] equals to  32.435*10^-6 [sec]
METER_CONST_PER_SEC_LON = 49.334 * 10**(-3) # 1 [meter] equals to  49.334*10^-3 [sec]

""" Degre:Minute format conversion to Degree:Minute:Second format"""
def DD2DMS(DD_value):
    DMS_deg = 0.0
    DMS_min = 0.0
    DMS_sec = 0.0
    sign = 1

    if(0 > DD_value):
        sign = -1

    DMS_deg = int(abs(DD_value)) 
    DMS_min = int((abs(DD_value) - DMS_deg) * 60) 
    DMS_sec = ((abs(DD_value) - DMS_deg) - (DMS_min/60.0)) * 3600.0

    return sign, DMS_deg, DMS_min, DMS_sec


""" Degree:Minute:Second format conversion to Degre:Minute format"""
def DMS2DD(sign, DMS_deg, DMS_min, DMS_sec):
    return (sign * (DMS_deg + (DMS_min/60) + (DMS_sec/3600)))

def print_DMS(vehicle):

    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon
    alt = vehicle.location.global_frame.alt

    lat_DMS = DD2DMS(lat)
    lon_DMS = DD2DMS(lon)

    print("lat: sign: %d, deg: %f, min: %f, sec: %f" % lat_DMS)
    print("lon: sign: %d, deg: %f, min: %f, sec: %f" % lon_DMS)


def diff_in_meter(sec_dest, sec_current, lat_lon_selector = 0):
    
    if(0 == lat_lon_selector):
        divider = METER_CONST_PER_SEC_LAT
    elif(1 == lat_lon_selector):
        divider = METER_CONST_PER_SEC_LON
    else:
        raise Exception("Given lat_lon_selector is invalid!!!!") 

    return (abs(sec_dest-sec_current) / divider)

def get_distance_metres_mod(aLocation1, aLocation2):

    dlat_DD = aLocation2.lat - aLocation1.lat
    dlon_DD = aLocation2.lon - aLocation1.lon

    dlat_DMS = DD2DMS(dlat_DD)
    dlon_DMS = DD2DMS(dlon_DD)

    dlat_meter = dlat_DMS[3]/METER_CONST_PER_SEC_LAT
    dlon_meter = dlon_DMS[3]/METER_CONST_PER_SEC_LON

    return math.sqrt((dlat_meter*dlat_meter) + (dlon_meter*dlon_meter)) 
    

def get_location_metres_mod(original_location, dNorth, dEast):

    original_location_DMS_Lat = DD2DMS(original_location.lat)
    original_location_DMS_Lon = DD2DMS(original_location.lon)

    dLat_sec = dNorth * METER_CONST_PER_SEC_LAT
    dLon_sec = dEast  * METER_CONST_PER_SEC_LON

    newlat = DMS2DD(original_location_DMS_Lat[0], original_location_DMS_Lat[1], original_location_DMS_Lat[2], original_location_DMS_Lat[3] + dLat_sec)
    newlon = DMS2DD(original_location_DMS_Lon[0], original_location_DMS_Lon[1], original_location_DMS_Lon[2], original_location_DMS_Lon[3] + dLon_sec)

    newlat = round(newlat, 7)
    newlon = round(newlon, 7)

    # targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt)

    if type(original_location) is LocationGlobal:
        targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation




# LocationGlobal:lat=-35.3632619,lon=149.1652376,alt=586.02
# LocationGlobal:lat=-35.3632158,lon=149.1652353,alt=585.97



#  LocationGlobal:lat=-35.3632162,lon=149.165235,alt=585.96
#  LocationGlobal:lat=-35.3632158,lon=149.1652353,alt=585.97

# lat=-35.3632159,lon=149.1652354,alt=585.96
# lat=-35.3632157,lon=149.1652344,alt=585.96

# print("diff lat: %f" % diff_in_meter(-35.3632065, -35.363204703369426))
# print("diff lon: %f" % diff_in_meter(149.1652353, 149.1652353, 1))
# print("diff alt: %f" % diff_in_meter(586.02, 585.97))

# print("\n1m")
# print("diff lat: %f" % diff_in_meter(-35.3632513, -35.36324231684716))
# print("diff lon: %f" % diff_in_meter(149.1652354, 149.1652354, 1))

# print("\n0.5m")
# print("diff lat: %f" % diff_in_meter(-35.3632418, -35.36323730842358))
# print("diff lon: %f" % diff_in_meter(149.1652354, 149.1652354, 1))

# print("\n0.2m")
# print("diff lat: %f" % diff_in_meter(-35.3632353, -35.36323350336943))
# print("diff lon: %f" % diff_in_meter(149.1652354, 149.1652354, 1))

# print("\n0.2m end")
# print("diff lat: %f" % diff_in_meter(-35.3632321, -35.36323350336943))
# print("diff lon: %f" % diff_in_meter(149.1652354, 149.1652354, 1))



# def haversine(lat1, lon1, lat2, lon2):
#     R = 6371.0  # Earth radius in kilometers

#     # Convert latitude and longitude from degrees to radians
#     lat1_rad = math.radians(lat1)
#     lon1_rad = math.radians(lon1)
#     lat2_rad = math.radians(lat2)
#     lon2_rad = math.radians(lon2)

#     # Haversine formula
#     dlon = lon2_rad - lon1_rad
#     dlat = lat2_rad - lat1_rad
#     a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
#     c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
#     distance = R * c

#     return distance * 1000  # Convert distance to meters

# # Coordinates of pose1 and pose2
# pose1 = {"lat": -35.3632321, "lon": 149.1652354}
# pose2 = {"lat": -35.36323350336943, "lon": 149.1652354}

# # Calculate distance
# distance = haversine(pose1["lat"], pose1["lon"], pose2["lat"], pose2["lon"])
# print("Distance between pose1 and pose2:", distance, "meters")

# lat,lon 1 = vehicle pose
# lat,lon 2 = target pose
def haversine2(lat1, lon1, lat2, lon2):
    R = 6371.0  # Earth radius in kilometers

    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Haversine formula for latitude
    dlat_rad = lat2_rad - lat1_rad
    a_lat = math.sin(dlat_rad / 2)**2
    c_lat = 2 * math.atan2(math.sqrt(a_lat), math.sqrt(1 - a_lat))
    distance_lat = R * c_lat * 1000  # Convert distance to meters

    # Haversine formula for longitude
    dlon_rad = lon2_rad - lon1_rad
    a_lon = math.sin(dlon_rad / 2)**2
    c_lon = 2 * math.atan2(math.sqrt(a_lon), math.sqrt(1 - a_lon))
    distance_lon = R * c_lon * 1000  # Convert distance to meters

    lat_sing = 1 if((lat2 - lat1) > 0) else -1
    lon_sing = 1 if((lon2 - lon1) > 0)  else -1

    return (distance_lat * lat_sing), (distance_lon * lon_sing)

# # # Coordinates of pose1 and pose2
# pose1 = {"lat": -35.3632619, "lon": 149.1652377}
# pose2 = {"lat": -35.36325291684716, "lon": 149.1652377}

# # Calculate distance
# distance_lat, distance_lon = haversine2(pose1["lat"], pose1["lon"], pose2["lat"], pose2["lon"])
# print("Distance in latitude:", distance_lat, "meters")
# print("Distance in longitude:", distance_lon, "meters")


def haversine3(lat1, lon1, lat2, lon2):
    R = 6371.0  # Earth radius in kilometers

    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Haversine formula for latitude
    dlat_rad = lat2_rad - lat1_rad
    a_lat = math.sin(dlat_rad / 2)**2
    c_lat = 2 * math.atan2(math.sqrt(a_lat), math.sqrt(1 - a_lat))
    distance_lat = R * c_lat * 1000  # Convert distance to meters
    direction_lat = "north" if dlat_rad > 0 else "south"

    # Haversine formula for longitude
    dlon_rad = lon2_rad - lon1_rad
    a_lon = math.sin(dlon_rad / 2)**2
    c_lon = 2 * math.atan2(math.sqrt(a_lon), math.sqrt(1 - a_lon))
    distance_lon = R * c_lon * 1000  # Convert distance to meters
    direction_lon = "east" if dlon_rad > 0 else "west"

    return distance_lat, direction_lat, distance_lon, direction_lon

# # Coordinates of pose1 and pose2
# pose2 = {"lat": -35.3632513, "lon": 149.1652354}
# pose1 = {"lat": -35.36324231684716, "lon": 149.1652354}

# # Calculate distance and direction
# distance_lat, distance_lon = haversine2(pose1["lat"], pose1["lon"], pose2["lat"], pose2["lon"])
# print("Distance in latitude:", abs(distance_lat), "meters")
# print("Distance in longitude:", abs(distance_lon), "meters")

# print("Distance in latitude:", abs(distance_lat), "meters", direction_lat)
# print("Distance in longitude:", abs(distance_lon), "meters", direction_lon)