#!/usr/bin/env python3
import sys
import os 
import datetime
import json
import random

# scripts relative path added 
sys.path.append(os.path.dirname(__file__))





class Measurement:
    def __init__(self, droneID):
        
        self.dirname = os.path.dirname(__file__)        
        self.path = os.path.join(self.dirname, f"measurement_files/drone_{droneID}_measurements")
        self.droneID = droneID       
        self.measured_datas = []   
        self.measurement_ID = 0

        # Create a folder which name is a timestamp
        time_object = datetime.datetime.now()
        object_creation_time = time_object.timestamp()
        datetime_object = datetime.datetime.fromtimestamp(object_creation_time)
        formatted_datetime = datetime_object.strftime("%Y_%m_%d_%H_%M_%S")
        self.full_path = os.path.join(self.dirname, self.path, formatted_datetime)

        if not (os.path.exists(os.path.join(self.dirname, self.path))):
             os.makedirs(os.path.join(self.dirname, self.path))

        os.makedirs(self.full_path) 

        self.path = os.path.join(self.full_path, f"drone_{droneID}_measurement.json")


    # Save measured values to a JSON object
    #                                          [˚C]       [%]    [ppm] 
    def store_measurement_datas_dict(self, temperature, Humidity, co2):   
        dict = {        
        "measurement_ID" : self.measurement_ID,
        "temperature": temperature,
        "Humidity": Humidity,
        "co2": co2,
        "droneID" : self.droneID        
        }
        self.measured_datas.append(json.dumps(dict))
        self.measurement_ID += 1   

    # Save measurement list to a json file
    def write_poses_to_file(self):
            # Writing to sample.json
            with open(self.path, "w") as outfile:
                for measured_data in self.measured_datas:                
                    outfile.write(measured_data)
                    outfile.write("\n")



# # Test
# test = Measurement(1)
# test.store_measurement_datas_dict(20,80,300)
# test.store_measurement_datas_dict(30,70,400)
# test.write_poses_to_file()

# random.seed()
# co2_random = random.randint(300, 400)
# hum_random = random.randint(50, 90)
# temp_random = random.random()
# print(co2_random)
# print(hum_random)
# print(temp_random*2 + 20)

