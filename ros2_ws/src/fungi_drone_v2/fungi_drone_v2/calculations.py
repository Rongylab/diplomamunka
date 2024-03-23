
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

print("diff lat: %f" % diff_in_meter(-35.3632159, -35.3632157))
print("diff lon: %f" % diff_in_meter(149.1652354, 149.1652344, 1))
# print("diff alt: %f" % diff_in_meter(586.02, 585.97))

