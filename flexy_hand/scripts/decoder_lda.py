#!/usr/bin/env python3
import rospy
from flexy_hand.msg import FingersAngles
from flexy_hand.msg import EMGdata
import joblib
import pandas as pd

# Load model
model_thumb = joblib.load('src/flexy_hand/scripts/model/lda_thumb.pkl')
model_index = joblib.load('src/flexy_hand/scripts/model/lda_index.pkl')
model_long = joblib.load('src/flexy_hand/scripts/model/lda_long.pkl')
model_ring = joblib.load('src/flexy_hand/scripts/model/lda_ring.pkl')
model_small = joblib.load('src/flexy_hand/scripts/model/lda_small.pkl')

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

    #X = [m1, m2, m3 , m4]

    # Make prediction
    y_thumb = model_thumb.predict(X)
    y_index = model_index.predict(X)
    y_long = model_long.predict(X)
    y_ring = model_ring.predict(X)
    y_small = model_small.predict(X)


    # Publish to the angles topic
    pub = rospy.Publisher('fingers_angles', FingersAngles, queue_size=10)
    
    # Create angles message
    msg = FingersAngles()
    if int(y_thumb) >= 0:
        if int(y_thumb) == 0:
            msg.thumb = 101
        else:
            msg.thumb = 179
    else:
        msg.thumb = 180
    if int(y_index) >= 0:
        if int(y_index) == 0:
            msg.index = 81
        else:
            msg.index = 179
    else:
        msg.index = 180
    if int(y_long) >= 0:
        if int(y_long) == 0:
            msg.long = 81
        else:
            msg.long = 179
    else:
        msg.long = 180
    if int(y_ring) >= 0:
        if int(y_ring) == 0:
            msg.ring = 81
        else:
            msg.ring = 179
    else:
        msg.ring = 180
    if int(y_small) >= 0:
        if int(y_small) == 0:
            msg.small = 81
        else:
            msg.small = 179
    else:
        msg.small = 180

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