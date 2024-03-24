import json
import os


class Positions:
    def __init__(self, init_from_file = False):
        self.positions = []
        self.pose_ID = 0
        self.dirname = os.path.dirname(__file__)
        self.path = os.path.join(self.dirname, "pose_files/pose.json")        

        if not (os.path.exists(os.path.join(self.dirname, "pose_files"))):
             os.makedirs(os.path.join(self.dirname, "pose_files"))
        elif (init_from_file):
            # print("Init from file:")
            self.read_poses_from_file()
    
    def store_coordinates_dict(self, lat, lon, alt, orientation = 0):
        dict = {
        "pose_ID" : self.pose_ID,
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "orientation": orientation
        }
        self.positions.append(json.dumps(dict)) #,  indent = 5))
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

# y = Positions(False)
# # print(y.get_pose_ID())

# y.store_coordinates_dict(1, 2, 3)
# y.store_coordinates_dict(4, 5, 6)
# y.write_poses_to_file()

# y = Positions(True)
# # y.read_poses_from_file()
# print(y.get_pose(0))
# print(y.get_pose(1))

# y.store_coordinates_dict(7, 8, 9)
# y.store_coordinates_dict(10, 11, 12)
# y.write_poses_to_file()

# pose = y.get_pose(1)
# print("lat: %f" %pose["lat"])
# print("lon: %f" %pose["lon"])
# print("alt: %f" %pose["alt"])

# print("")
# print(y.get_pose(1)["lat"])
