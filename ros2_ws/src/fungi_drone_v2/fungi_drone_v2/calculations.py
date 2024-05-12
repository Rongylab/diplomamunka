
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
from geographiclib.geodesic import Geodesic
from decimal import Decimal, getcontext
import navpy


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


# lat,lon 1 = vehicle pose
# lat,lon 2 = target pose
def sign_difference_in_meters(lat1, lon1, lat2, lon2):
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

    # return (round((distance_lat * lat_sing), 4), (round((distance_lon * lon_sing), 4)))
    return (distance_lat * lat_sing), (distance_lon * lon_sing)


def meters_to_difference(base_lat, base_lon, meters_lat, meters_lon):
    R = 6371.0  # Earth radius in kilometers

    # Convert base latitude and longitude from degrees to radians
    base_lat_rad = math.radians(base_lat)
    base_lon_rad = math.radians(base_lon)

    # Convert distance from meters to kilometers
    meters_lat_km = meters_lat / 1000
    meters_lon_km = meters_lon / 1000

    # Convert latitude difference to radians
    dlat_rad = meters_lat_km / R
    # Convert longitude difference to radians
    dlon_rad = meters_lon_km / (R * math.cos(base_lat_rad)) 

    # Calculate new latitude in radians
    new_lat_rad = base_lat_rad + dlat_rad
    # Calculate new longitude in radians
    new_lon_rad = base_lon_rad + dlon_rad

    # Convert new latitude and longitude from radians to degrees
    new_lat = math.degrees(new_lat_rad)
    new_lon = math.degrees(new_lon_rad)

    return new_lat, new_lon



def distance_and_directions(lat1, lon1, lat2, lon2):
    R = 6371.0  # Earth radius in kilometers

    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Calculate differences in latitude and longitude
    dlat_rad = lat2_rad - lat1_rad
    dlon_rad = lon2_rad - lon1_rad

    # Haversine formula for distance
    a = math.sin(dlat_rad / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon_rad / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c * 1000  # Convert distance to meters

    # Convert differences to meters
    distance_lat = dlat_rad * R * 1000
    distance_lon = dlon_rad * R * 1000 * math.cos((lat1_rad + lat2_rad) / 2)  # Correct for latitude

    return distance, distance_lat, distance_lon



def NED_to_meterDiff(lat0, lon0, lat1, lon1):
    const_minute = 1851.8

    lat0 *= 60 
    lon0 *= 60 
    lat1 *= 60 
    lon1 *= 60 

    lat0_m = lat0 * const_minute
    lon0_m = lon0 * const_minute

    lat1_m = lat1 * const_minute
    lon1_m = lon1 * const_minute

    diff_x = (lat0_m-lat1_m)
    diff_y = (lon0_m - lon1_m)

    return diff_x, diff_y

def haver(lat1, lon1, lat2, lon2):

    sign = 1
    R = 6371e3
    f1 = lat1 * math.pi/180
    f2 = lat2 * math.pi/180
    df = (lat2-lat1) * math.pi/180
    dlamda = (lon2-lon1) * math.pi/180

    a = math.sin(df/2) * math.sin(df/2) + math.cos(f1) * math.cos(f2) * math.sin(dlamda/2) * math.sin(dlamda/2)

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c

    if(lat1 != lat2):
        if((lat2-lat1) < 0):
            sign *= -1
    
    if(lon1 != lon2):
        if((lon2-lon1) < 0):
            sign *= -1
    
    return (d * sign)



# LocationGlobal:lat=-35.3632619,lon=149.1652376,alt=586.02
# LocationGlobal:lat=-35.3632158,lon=149.1652353,alt=585.97



#  LocationGlobal:lat=-35.3632162,lon=149.165235,alt=585.96
#  LocationGlobal:lat=-35.3632158,lon=149.1652353,alt=585.97

# lat=-35.3632159,lon=149.1652354,alt=585.96
# lat=-35.3632157,lon=149.1652344,alt=585.96

# print("diff lat: %f" % diff_in_meter(-35.3632159, -35.3632157))
# print("diff lon: %f" % diff_in_meter(149.1652354, 149.1652344, 1))
# print("diff alt: %f" % diff_in_meter(586.02, 585.97))





# GAP START

#lat=-35.3632627,lon=149.165077,alt=588.6


# # # # Define the positions of Pose0 and Pose1
# pose0_lat, pose0_lon, pose0_alt = -35.3632621, 149.1650666, 585.19
# pose1_lat, pose1_lon, pose1_alt = -35.3632627, 149.165077, 586.0

# pose0_lat, pose0_lon, pose0_alt = 47.49801, 18.99388, 585.19

# pose1_lat, pose1_lon, pose1_alt = 47.49802, 18.99392, 586.0

# # #pose1_lat, pose1_lon, pose1_alt = -35.3632619, 149.1652376, 586.0
# # # pose1_lat, pose1_lon, pose1_alt = -35.3632944, 149.1653309, 586.0

# pose1_lat, pose1_lon, pose1_alt = -35.3632874, 149.165413, 586.0 # --> lon irányba 2.347154750570013m diffi #1653309
# # pose1_lat, pose1_lon, pose1_alt = -35.363262, 149.1653309, 586.0 
# # # pose1_lat, pose1_lon, pose1_alt = -35.3632944, 149.1652376, 586.0



# # a = LocationGlobal(pose0_lat, pose0_lon, pose0_alt)
# # b = LocationGlobal(pose1_lat, pose1_lon, pose1_alt)



# diff_x, diff_y = sign_difference_in_meters(pose0_lat, pose0_lon, pose1_lat, pose1_lon)
# # distance, diff_x, diff_y = distance_and_directions(pose0_lat, pose0_lon, pose1_lat, pose1_lon)
# # diff_z = pose0_alt - pose1_alt


# diff_x, diff_y = NED_to_meterDiff(pose0_lat, pose0_lon, pose1_lat, pose1_lon)

# d_lon = haver(pose0_lat, pose0_lon, pose0_lat, pose1_lon)
# d_lat = haver(pose0_lat, pose0_lon, pose1_lat, pose0_lon)

# print("d_lat: ", d_lat)
# print("d_lon: ", d_lon)
# # # print(" c", math.sqrt((d_lat * d_lat) + (d_lon * d_lon)))

# # d_lat = 0.0
# # d_lon = 31.775

# new_lat, new_lon = meters_to_difference(pose0_lat, pose0_lon, d_lat, d_lon)

# print("new_lat", new_lat)
# print("new_lon", new_lon)


# print("diff_x", diff_x)
# print("diff_y", diff_y)
# print(" c", math.sqrt((diff_x * diff_x) + (diff_y * diff_y)))
# # print("diff_z", diff_z)



# new_lat, new_lon = meters_to_difference(pose0_lat, pose0_lon, diff_x, diff_y) #- (2.347154750570013 - 0.5310272793401793 + 0.12014119478243046 - 0.02718110503030502 + 0.00614953483868419 - 0.001391287540286612)) #- 2.347154750570013
# # # c = get_location_metres(a, diff_x, diff_y)

# print("new_lat: ", new_lat)
# print("new_lon: ", new_lon)

# # print("Diff in degree LAT: ", (pose1_lat - new_lat))
# # print("Diff in degree LON: ", (pose1_lon - new_lon))

# # # print("new_lat: ", c.lat)
# # # print("new_lon: ", c.lon)

# diff_x, diff_y = sign_difference_in_meters(pose1_lat, pose1_lon, new_lat, new_lon) ## EZ AZ FV jóóóóóóóóó!!!!
# # # diff_x, diff_y = sign_difference_in_meters(pose1_lat, pose1_lon, c.lat, c.lon)
# # # distance, diff_x, diff_y = distance_and_directions(pose0_lat, pose0_lon, new_lat, new_lon)
# print("new diff_x with pose1", diff_x)
# print("new diff_y with pose1", diff_y)

# diff_x, diff_y = sign_difference_in_meters(pose0_lat, pose0_lon, new_lat, new_lon)
# print("new diff_x with Base pose", diff_x)
# print("new diff_y with Base pose", diff_y)




# ned = [1, 0, 0]
# lat_ref, lon_ref, alt_ref = pose0_lat, pose0_lon, pose0_alt  # deg, meters
# ecef = navpy.ned2ecef(ned, lat_ref, lon_ref, alt_ref)
# print("NED:", ned)
# print("ECEF:", ecef)
# print("Notice that 'down' is not same as 'ecef-z' coordinate.")

#GAP end






# # Calculate the differences in latitude, longitude, and altitude
# delta_lat = pose1_lat - pose0_lat
# delta_lon = pose1_lon - pose0_lon
# delta_alt = pose1_alt - pose0_alt

# # Calculate the North and East displacements using geographiclib
# geod = Geodesic.WGS84
# g = geod.Inverse(pose0_lat, pose0_lon, pose1_lat, pose1_lon)
# delta_n = g['lat2'] - g['lat1']
# delta_e = g['lon2'] - g['lon1']

# # Convert angular displacements to meters
# meters_per_degree_lat = 111111
# meters_per_degree_lon = 111111 * abs(pose0_lon) / 180

# # Calculate the displacements in meters
# delta_n_meters = delta_n * meters_per_degree_lat
# delta_e_meters = delta_e * meters_per_degree_lon

# print("North displacement (meters):", delta_n_meters)
# print("East displacement (meters):", delta_e_meters)
# print("Down displacement (meters):", delta_alt)


# 149.1653520084698




# def distance_and_directions(lat1, lon1, lat2, lon2):
#     # Set decimal precision to avoid floating-point errors
#     getcontext().prec = 28

#     R = Decimal('6371.0')  # Earth radius in kilometers

#     # Convert latitude and longitude from degrees to radians
#     lat1_rad = Decimal(math.radians(lat1))
#     lon1_rad = Decimal(math.radians(lon1))
#     lat2_rad = Decimal(math.radians(lat2))
#     lon2_rad = Decimal(math.radians(lon2))

#     # Calculate differences in latitude and longitude
#     dlat_rad = lat2_rad - lat1_rad
#     dlon_rad = lon2_rad - lon1_rad

#     # Haversine formula for distance
#     a = Decimal(math.sin(dlat_rad / Decimal('2')))**2 + Decimal(math.cos(lat1_rad)) * Decimal(math.cos(lat2_rad)) * Decimal(math.sin(dlon_rad / Decimal('2')))**2
#     c = Decimal('2') * Decimal(math.atan2(Decimal(math.sqrt(a)), Decimal(math.sqrt(Decimal('1') - a))))
#     distance = R * c * Decimal('1000')  # Convert distance to meters

#     # Convert differences to meters
#     distance_lat = dlat_rad * R * Decimal('1000')
#     distance_lon = dlon_rad * R * Decimal('1000') * Decimal(math.cos((lat1_rad + lat2_rad) / Decimal('2')))  # Correct for latitude

#     return distance, distance_lat, distance_lon



# def meters_to_difference(base_lat, base_lon, meters_lat, meters_lon):
#     # Set decimal precision to avoid floating-point errors
#     getcontext().prec = 28

#     R = Decimal('6371.0')  # Earth radius in kilometers

#     # Convert base latitude and longitude from degrees to radians
#     base_lat_rad = Decimal(math.radians(base_lat))
#     base_lon_rad = Decimal(math.radians(base_lon))

#     # Convert distance from meters to kilometers
#     meters_lat_km = Decimal(meters_lat) / Decimal('1000')
#     meters_lon_km = Decimal(meters_lon) / Decimal('1000')

#     # Convert latitude difference to radians
#     dlat_rad = meters_lat_km / R
#     # Convert longitude difference to radians
#     dlon_rad = meters_lon_km / (R * Decimal(math.cos(base_lat_rad)))

#     # Calculate new latitude in radians
#     new_lat_rad = base_lat_rad + dlat_rad
#     # Calculate new longitude in radians
#     new_lon_rad = base_lon_rad + dlon_rad

#     # Convert new latitude and longitude from radians to degrees
#     new_lat = Decimal(math.degrees(new_lat_rad))
#     new_lon = Decimal(math.degrees(new_lon_rad))

#     return new_lat, new_lon


# pose0_lat, pose0_lon, pose0_alt = -35.363262, 149.165079, 586.0
# pose1_lat, pose1_lon, pose1_alt = -34.363262, 148.1650300, 586.0


# distance, distance_lat, distance_lon = distance_and_directions(pose0_lat, pose0_lon, pose1_lat, pose1_lon)
# new_lat, new_lon = meters_to_difference(pose0_lat, pose0_lon, distance_lat, distance_lon)

# distance, distance_lat, distance_lon = distance_and_directions(pose1_lat, pose1_lon, new_lat, new_lon)

# print("distance_lat", distance_lat)
# print("distance_lon", distance_lon)







# def lat_lon_to_xy(lat, lon, origin_lat, origin_lon):
#     # Earth radius in kilometers
#     R = 6371.0

#     # Convert latitude and longitude from degrees to radians
#     lat_rad = math.radians(lat)
#     lon_rad = math.radians(lon)
#     origin_lat_rad = math.radians(origin_lat)
#     origin_lon_rad = math.radians(origin_lon)

#     # Calculate differences in latitude and longitude
#     dlat_rad = lat_rad - origin_lat_rad
#     dlon_rad = lon_rad - origin_lon_rad

#     # Haversine formula for distance
#     a = math.sin(dlat_rad / 2)**2 + math.cos(origin_lat_rad) * math.cos(lat_rad) * math.sin(dlon_rad / 2)**2
#     c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
#     distance = R * c * 1000  # Convert distance to meters

#     # Calculate x and y values in meters
#     x = distance * math.cos(origin_lat_rad) * math.cos(dlon_rad) * 1000
#     y = distance * math.cos(origin_lat_rad) * math.sin(dlon_rad) * 1000

#     return x, y


# def xy_to_lat_lon(origin_lat, origin_lon, x_diff, y_diff):
#     # Earth radius in kilometers
#     R = 6371.0

#     # Convert origin latitude and longitude from degrees to radians
#     origin_lat_rad = math.radians(origin_lat)
#     origin_lon_rad = math.radians(origin_lon)

#     # Calculate new latitude and longitude differences
#     lat_diff = y_diff / (R * 1000)
#     lon_diff = x_diff / (R * 1000 * math.cos(origin_lat_rad))

#     # Calculate new latitude and longitude in radians
#     new_lat_rad = origin_lat_rad + lat_diff
#     new_lon_rad = origin_lon_rad + lon_diff

#     # Convert new latitude and longitude from radians to degrees
#     new_lat = math.degrees(new_lat_rad)
#     new_lon = math.degrees(new_lon_rad)

#     return new_lat, new_lon



# pose0_lat, pose0_lon, pose0_alt = -35.3632619, 149.1652376, 586.0
# pose1_lat, pose1_lon, pose1_alt = -35.3632619, 148.1652376, 586.0
# # pose1_lat, pose1_lon, pose1_alt = -35.3632619, 149.165413, 586.0

# x, y = lat_lon_to_xy(pose1_lat, pose1_lon, pose0_lat, pose0_lon)
# print("x: ", x)
# print("y: ", y)