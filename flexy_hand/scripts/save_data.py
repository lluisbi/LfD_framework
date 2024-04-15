#!/usr/bin/env python3
import rospy
from flexy_hand.msg import FingersAngles
from flexy_hand.msg import EMGdata
import csv
import time
import os

MVC = []

def load_mvc():
    # Open the text file in read mode
    with open('src/flexy_hand/scripts/data/mvc.txt', 'r') as file:
        # Read the lines of the file
        lines = file.readlines()

        # Iterate through each line in the file
        for line in lines:
            # Split the line into values using comma as delimiter
            values = line.strip().split(',')
            
            # Add the values to the list
            MVC.append(values)

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
        data = ['thumb', 'index', 'long', 'ring', 'small', 'muscle1', 'muscle2', 'muscle3', 'muscle4']
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
        self.muscle1 = 0
        self.muscle2 = 0
        self.muscle3 = 0
        self.muscle4 = 0
        self.filename = filename

    def ang_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard: \n%s", data)
        self.thumb = data.thumb
        self.index = data.index
        self.long  = data.long
        self.ring  = data.ring
        self.small = data.small

    def emg_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard: \n%s", data)
        self.muscle1 = data.muscle1*100/float(MVC[0][0])
        self.muscle2 = data.muscle2*100/float(MVC[0][1])
        self.muscle3 = data.muscle3*100/float(MVC[0][2])
        self.muscle4 = data.muscle4*100/float(MVC[0][3])

    def save(self):
        with open(os.path.join("src/flexy_hand/scripts/data", self.filename), 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            data = [self.thumb, self.index, self.long, self.ring, self.small, self.muscle1, self.muscle2, self.muscle3, self.muscle4]
            writer.writerow(data)
    
def listener():

    rospy.init_node('data_saver', anonymous=False)

    filename = create_csv_file('data')
    saver = Saver(filename)

    rospy.Subscriber("fingers_angles", FingersAngles, saver.ang_callback)
    rospy.Subscriber("emg", EMGdata, saver.emg_callback)

    prev_time = time.time()
    elapsed_time = 0.1 # seconds
    while not rospy.is_shutdown():
        if time.time() - prev_time > elapsed_time:
            saver.save()
            prev_time = time.time()

    rospy.spin() # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    load_mvc()
    listener()