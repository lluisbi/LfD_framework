#!/usr/bin/env python3
import rospy
from flexy_hand.msg import FingersAngles
from flexy_hand.msg import EMGdata
import joblib
import pandas as pd

# Load model
model = joblib.load('src/flexy_hand/scripts/model/lr_model_full.pkl')

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

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard: \n%s", data)

    m1 = data.muscle1*100/float(MVC[0][0])
    m2 = data.muscle2*100/float(MVC[0][1])
    m3 = data.muscle3*100/float(MVC[0][2])
    m4 = data.muscle4*100/float(MVC[0][3])

    # Convert data format
    X = pd.DataFrame([[m1,m2,m3,m4]],columns=['muscle1','muscle2','muscle3','muscle4'])
    
    # Make prediction
    y = model.predict(X)

    # Publish to the angles topic
    pub = rospy.Publisher('fingers_angles', FingersAngles, queue_size=10)
    
    # Create angles message
    msg = FingersAngles()
    if y[0][0] >= 0:
        msg.thumb = round(y[0][0])
    elif y[0][0] < 180:
        msg.thumb = 180
    else:
        msg.thumb = 0
    if y[0][1] >= 0:
        msg.index = round(y[0][1])
    elif y[0][1] < 180:
        msg.thumb = 180
    else:
        msg.index = 0
    if y[0][2] >= 0:
        msg.long = round(y[0][2])
    elif y[0][1] < 180:
        msg.thumb = 180
    else:
        msg.long = 0
    if y[0][3] >= 0:
        msg.ring = round(y[0][3])
    elif y[0][3] < 180:
        msg.thumb = 180
    else:
        msg.ring = 0
    if y[0][4] >= 0:
        msg.small = round(y[0][4])
    elif y[0][5] < 180:
        msg.thumb = 180
    else:
        msg.small = 0

    rospy.loginfo(msg)

    # Send message
    pub.publish(msg)
    
def listener():
    # ROS node
    rospy.init_node('decoder', anonymous=False)
    rospy.Subscriber("emg", EMGdata, callback)
    rospy.spin() # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    load_mvc()
    listener()