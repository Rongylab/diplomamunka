import json
import os


class Positions:
    def __init__(self, init_from_file = False):
        self.positions = []
        self.pose_ID = 0
        self.path = os.path.basename("/pose_files/pose.json")

        if not (os.path.exists("pose_files")):
             os.makedirs("pose_files") 
        elif (init_from_file):
            print("Init from file:")   # TODO!! Megírni az init from file részt!!! 
    
    def store_coordinates_dict(self, lat, lon, alt, orientation = 0):
        dict = {
        "pose_ID" : self.pose_ID,
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "orientation": orientation
        }
        self.positions.append(json.dumps(dict,  indent = 5))
        self.pose_ID += 1

    def get_positions(self):
        return self.positions
    
    def get_pose(self, pose_ID):
        return self.positions[pose_ID]
    
    def set_pose_ID(self, new_pose_ID):
        self.pose_ID = new_pose_ID

    def get_pose_ID(self):
        return self.pose_ID
        
    def write_poses_to_file(self):
        # Writing to sample.json
        with open(self.path, "w") as outfile:
            for pose in self.positions:                
                outfile.write(pose)

    def write_one_pose_to_file(self, index):
       # Writing to sample.json
       with open(self.path, "a") as outfile:
                outfile.write(self.positions[index])

 # TODO! Megírni a visszaolvasást 
    def read_poses_from_file(self):
        # Opening JSON file
        with open(self.path, 'r') as openfile: 
            # Reading from json file
            json_object = json.load(openfile)


# def store_coordinates_dict(pose_ID, lat, lon, alt, orientation = 0):
#     dict = {
#     "pose_ID" : pose_ID,
#     "lat": lat,
#     "lon": lon,
#     "alt": alt,
#     "orientation": orientation
#     }
#     return dict