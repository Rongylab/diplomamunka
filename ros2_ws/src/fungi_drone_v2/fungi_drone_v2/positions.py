import json
import os

#TEST
import calculations as calc
from dronekit import LocationGlobal
import datetime
#TEST


class Positions:
    def __init__(self, droneID, baseFrame, init_from_file = False, path = "" ):
        self.positions = []
        self.pose_ID = 0
        self.dirname = os.path.dirname(__file__)
        self.baseFrame = baseFrame

        if(path != ""):
            self.path = os.path.join(path, f"drone_{droneID}_pose.json")
        else:
            self.path = os.path.join(self.dirname, f"pose_files/drone_{droneID}_pose.json")
        self.droneID = droneID   

        if not (os.path.exists(os.path.join(self.dirname, "pose_files"))):
             os.makedirs(os.path.join(self.dirname, "pose_files"))
        elif (init_from_file):
            # print("Init from file:")
            self.read_poses_from_file()
    
    def store_coordinates_dict(self, lat, lon, alt, saveImage_flag = 0 ,orientation = 0):

        # diff_x, diff_y = calc.sign_difference_in_meters(self.baseFrame.lat, self.baseFrame.lon, lat, lon)
        diff_x = calc.haver(self.baseFrame.lat, self.baseFrame.lon, lat, self.baseFrame.lon)
        diff_y = calc.haver(self.baseFrame.lat, self.baseFrame.lon, self.baseFrame.lat, lon)

        diff_alt = alt - self.baseFrame.alt

        dict = {
        "pose_ID" : self.pose_ID,
        "lat": diff_x,
        "lon": diff_y,
        "alt": diff_alt,
        "orientation": orientation,
        "droneID": self.droneID,
        "saveImage": saveImage_flag
        }
        self.positions.append(json.dumps(dict)) #,  indent = 5))
        self.pose_ID += 1

    def get_positions(self):
        return self.positions
    
    def get_pose(self, pose_ID):
        pose = json.loads(self.positions[pose_ID])
        new_lat, new_lon = calc.meters_to_difference(self.baseFrame.lat, self.baseFrame.lon, pose["lat"], pose["lon"])
        pose["alt"] = pose["alt"] + self.baseFrame.alt
        pose["lat"] = new_lat
        pose["lon"] = new_lon        
        return pose
    
    def set_pose_ID(self, new_pose_ID):
        self.pose_ID = new_pose_ID

    def get_pose_ID(self):
        return self.pose_ID
        
    def write_poses_to_file(self):
        # Writing to sample.json
        with open(self.path, "w") as outfile:
            for pose in self.positions:                
                outfile.write(pose)
                outfile.write("\n")

    def write_one_pose_to_file(self, index):
       # Writing to sample.json
       with open(self.path, "a") as outfile:
                outfile.write(self.positions[index])
 
    def read_poses_from_file(self):
        pose_ID_local = 0
        # Opening JSON file
        with open(self.path, 'r') as file:
            # Read the contents of the file
            data = file.read()
            
            # Split the file contents by newline character
            json_objects = data.strip().split('\n')
            
            # Iterate over each JSON object
            for json_object in json_objects:
                # Load the JSON object
                obj = json.loads(json_object)                
                self.positions.append(json.dumps(obj))
                pose_ID_local += 1

            self.pose_ID = pose_ID_local                   
                
    def get_imageSave_flag(self, desired_pose_ID = 0):        
        dict = json.loads(self.positions[desired_pose_ID])
        return (dict["saveImage"])



        # for key, value in self.positions.items():
        #     if key == "pose_ID" and value == desired_pose_ID:
        #         save_image_flag = self.positions["saveImage"]
        #         break
        # return (save_image_flag)


class DebugPoseSaver:
    def __init__(self, droneID, baseFrame):
        
        self.dirname = os.path.dirname(__file__)        
        self.path = os.path.join(self.dirname, f"debug/drone_{droneID}_debugposes") # drone_{droneID}_images.json
        self.droneID = droneID          

        # Create a folder which name is a timestamp
        time_object = datetime.datetime.now()
        object_creation_time = time_object.timestamp()
        datetime_object = datetime.datetime.fromtimestamp(object_creation_time)
        formatted_datetime = datetime_object.strftime("%Y_%m_%d_%H_%M_%S")
        self.full_path = os.path.join(self.dirname, self.path, formatted_datetime)

        if not (os.path.exists(os.path.join(self.dirname, self.path))):
             os.makedirs(os.path.join(self.dirname, self.path))


        self.desiredPath = os.path.join(self.full_path, "desired")
        self.realPath = os.path.join(self.full_path, "real")

        os.makedirs(self.full_path)
        os.makedirs(self.desiredPath)
        os.makedirs(self.realPath)

        self.desiredpose_object = Positions(self.droneID, baseFrame, init_from_file = False, path = self.desiredPath)
        self.realpose_object = Positions(self.droneID, baseFrame, init_from_file = False, path = self.realPath)
        

    # Save an image to a folder
    def save_a_debug_pose(self, desired_pose, real_pose):        
        self.desiredpose_object.store_coordinates_dict(desired_pose.lat, desired_pose.lon, desired_pose.alt)  
        self.realpose_object.store_coordinates_dict(real_pose.lat, real_pose.lon, real_pose.alt)  
        
    # Save image list and their poses to a json file
    def save_poses_json(self):
        self.desiredpose_object.write_poses_to_file()
        self.realpose_object.write_poses_to_file()









# Test part

# def store_coordinates_dict(pose_ID, lat, lon, alt, orientation = 0):
#     dict = {
#     "pose_ID" : pose_ID,
#     "lat": lat,
#     "lon": lon,
#     "alt": alt,
#     "orientation": orientation
#     }
#     return dict            

# pose0_lat, pose0_lon, pose0_alt = -35.3632621, 149.1650666, 585.19
# pose1_lat, pose1_lon, pose1_alt = -35.3632622, 149.1650847, 586.0
# pose2_lat, pose2_lon, pose2_alt = -35.3632874, 149.165413, 586.0

# Base_loc = LocationGlobal(pose0_lat, pose0_lon, pose0_alt)
# loc1 = LocationGlobal(pose1_lat, pose1_lon, pose1_alt)
# loc2 = LocationGlobal(pose2_lat, pose2_lon, pose2_alt)

# # diff_x, diff_y = calc.sign_difference_in_meters(Base_loc.lat, Base_loc.lon, new_lat, new_lon)
# y = Positions(0, Base_loc, False)
# # # print(y.get_pose_ID())

# y.store_coordinates_dict(Base_loc.lat, Base_loc.lon, Base_loc.alt, 0)
# y.store_coordinates_dict(loc1.lat, loc1.lon, loc1.alt, 0)
# y.store_coordinates_dict(loc2.lat, loc2.lon, loc2.alt, 1)

# # y.store_coordinates_dict(4, 5, 6, 1)
# y.write_poses_to_file()

# # desired_pose_ID = 1
# # save_image_flag = None 

# a = y.get_pose(1)
# print(a)
# print(type(a))

# 

# save_image_flag = y.get_imageSave_flag(desired_pose_ID)
# print(save_image_flag)
# # # Print or use the saveImage_flag
# # if save_image_flag is not None:
# #     print("SaveImage flag for pose_ID", desired_pose_ID, ":", save_image_flag)
# # else:
# #     print("Pose ID", desired_pose_ID, "not found in the dictionary.")







# y.write_poses_to_file()



            

# from dronekit import LocationGlobal
# import json

# y = Positions(0, True)
# # y.read_poses_from_file()
# # print(y.get_pose(0))
# # print(y.get_pose(1))

# pose = y.get_pose(0)
# res = json.loads(pose)
# new_pose = LocationGlobal(res["lat"], res["lon"], res["alt"]) 
# print(new_pose)

# y.store_coordinates_dict(7, 8, 9)
# y.store_coordinates_dict(10, 11, 12)
# y.write_poses_to_file()

# pose = y.get_pose(1)
# print("lat: %f" %pose["lat"])
# print("lon: %f" %pose["lon"])
# print("alt: %f" %pose["alt"])

# print("")
# print(y.get_pose(1)["lat"])
