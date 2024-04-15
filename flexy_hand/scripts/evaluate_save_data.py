#!/usr/bin/env python3
import rospy
from flexy_hand.msg import FingersAngles
import csv
import os
import time

def create_csv_file(base_name):

    number = 0
    file_name = f"{base_name}{number}.csv"
    
    # Check if the file already exists, or increment the number
    while os.path.exists(os.path.join("src/flexy_hand/scripts/data", file_name)):
        number += 1
        file_name = f"{base_name}{number}.csv"
    
    # Create the CSV file
    with open(os.path.join("src/flexy_hand/scripts/data", file_name), 'w') as csvfile:
        writer = csv.writer(csvfile)
        data = ['thumb', 'index', 'long', 'ring', 'small', 'thumb_pred', 'index_pred', 'long_pred', 'ring_pred', 'small_pred']
        writer.writerow(data)
    
    print(f"File '{file_name}' has been created")

    return str(base_name)+str(number)+".csv"

class Saver:
    def __init__(self, filename):
        self.thumb = 0
        self.index = 0
        self.long  = 0
        self.ring  = 0
        self.small = 0
        self.thumb_pred = 0
        self.index_pred = 0
        self.long_pred  = 0
        self.ring_pred  = 0
        self.small_pred = 0
        self.filename = filename

    def ev_ang_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard: \n%s", data)
        self.thumb = data.thumb
        self.index = data.index
        self.long  = data.long
        self.ring  = data.ring
        self.small = data.small

    def ang_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard: \n%s", data)
        self.thumb_pred = data.thumb
        self.index_pred = data.index
        self.long_pred  = data.long
        self.ring_pred  = data.ring
        self.small_pred = data.small

    def save(self):
        with open(os.path.join("src/flexy_hand/scripts/data", self.filename), 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            data = [self.thumb_pred, self.index_pred, self.long_pred, self.ring_pred, self.small_pred, self.thumb, self.index, self.long, self.ring, self.small]
            writer.writerow(data)
    
def listener():

    rospy.init_node('data_saver', anonymous=False)

    filename = create_csv_file('ev_data')
    saver = Saver(filename)

    rospy.Subscriber("fingers_angles", FingersAngles, saver.ang_callback)
    rospy.Subscriber("evaluate_fingers_angles", FingersAngles, saver.ev_ang_callback)

    prev_time = time.time()
    elapsed_time = 0.1 # second
    while not rospy.is_shutdown():
        if time.time() - prev_time > elapsed_time:
            saver.save()
            prev_time = time.time()

    rospy.spin() # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()