#!/usr/bin/env python3
import sys
import os 
import time
import cv2
import datetime
import positions as pos

# scripts relative path added 
sys.path.append(os.path.dirname(__file__))





class ImageSaver:
    def __init__(self, droneID, baseFrame):
        
        self.dirname = os.path.dirname(__file__)        
        self.path = os.path.join(self.dirname, f"image_files/drone_{droneID}_images") # drone_{droneID}_images.json
        self.droneID = droneID          

        # Create a folder which name is a timestamp
        time_object = datetime.datetime.now()
        object_creation_time = time_object.timestamp()
        datetime_object = datetime.datetime.fromtimestamp(object_creation_time)
        formatted_datetime = datetime_object.strftime("%Y_%m_%d_%H_%M_%S")
        self.full_path = os.path.join(self.dirname, self.path, formatted_datetime)

        if not (os.path.exists(os.path.join(self.dirname, self.path))):
             os.makedirs(os.path.join(self.dirname, self.path))

        os.makedirs(self.full_path)

        self.pose_object = pos.Positions(self.droneID, baseFrame, init_from_file = False, path = self.full_path)

    # Save an image to a folder
    def save_an_image(self, img, pose):
        img_id = self.pose_object.get_pose_ID()
        self.pose_object.store_coordinates_dict(pose.lat, pose.lon, pose.alt)
        img_path = os.path.join(self.full_path, f"image_{img_id}")
        cv2.imwrite(img_path + ".png", img) 


    # Save image list and their poses to a json file
    def save_poses_json(self):
        self.pose_object.write_poses_to_file()


# test = ImageSaver(1)

# ct = datetime.datetime.now()
# ts = ct.timestamp()

# integer_timestamp = ts  # Example integer timestamp
# datetime_object = datetime.datetime.fromtimestamp(integer_timestamp)
# print("Datetime from Integer Timestamp:", datetime_object)

# formatted_datetime = datetime_object.strftime("%Y_%m_%d_%H_%M_%S")
# print("Formatted Datetime:", formatted_datetime)