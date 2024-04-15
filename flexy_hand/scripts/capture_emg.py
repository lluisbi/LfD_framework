#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import requests
from flexy_hand.msg import EMGdata

emg_1 = 0
emg_2 = 0
emg_3 = 0
emg_4 = 0

# Function to receive data from the EMG signals HTTP stream
def get_data():
    global emg_1, emg_2, emg_3, emg_4

    try:
        # HTTP GET request
        url = 'http://192.168.92.228:9220/samples' # Change the IP address if needed
        response = requests.get(url)

        # Check if the request was successful (status code 200)
        if response.status_code == 200:
            # Parse JSON data
            json_data = response.json()

            # Extract EMG signals from json
            emg_1 = np.mean(json_data['channels'][0]['samples'])
            emg_2 = np.mean(json_data['channels'][1]['samples'])
            emg_3 = np.mean(json_data['channels'][2]['samples'])
            emg_4 = np.mean(json_data['channels'][3]['samples'])

            return emg_1, emg_2, emg_3, emg_4
            
        else:
            print(f"Error: Unable to fetch data. Status code: {response.status_code}")
            return emg_1, emg_2, emg_3, emg_4
        
    except Exception as e:
        print(f"An error occurred: {e}")
        return emg_1, emg_2, emg_3, emg_4

# Animation function to be executed on each frame
def update_plot(frame):

    value1, value2, value3, value4 = get_data()

    msg = EMGdata()
    msg.muscle1 = value1
    msg.muscle2 = value2
    msg.muscle3 = value3
    msg.muscle4 = value4
    rospy.loginfo(msg)
    pub.publish(msg)

    # Update data in the graph
    x.append(frame)
    y1.append(value1)
    y2.append(value2)
    y3.append(value3)
    y4.append(value4)

    # Limit the number of points on the graph to keep the x-axis constant
    if len(x) > max_points:
        x.pop(0)
        y1.pop(0)
        y2.pop(0)
        y3.pop(0)
        y4.pop(0)
    
    # Clear axis
    plt.cla()
    
    # Plot data
    plt.plot(x, y1, label='EMG 1')
    plt.plot(x, y2, label='EMG 2')
    plt.plot(x, y3, label='EMG 3')
    plt.plot(x, y4, label='EMG 4')
    
    # Add legend
    plt.legend(loc='upper left')
    
    # Axis title and labels
    plt.title('Mean EMG  in Real Time')
    plt.xlabel('Time [s]')
    plt.ylabel('EMG [uV]')

# ROS Node
pub = rospy.Publisher('emg', EMGdata, queue_size=10)
rospy.init_node('emg_capturer', anonymous=False)

# Create figure and axis
fig, ax = plt.subplots()

# Maximum number of points to display on the graph
max_points = 60

# Lists to store data
x = []
y1 = []
y2 = []
y3 = []
y4 = []

# Create animation
ani = FuncAnimation(fig, update_plot, interval=50) # Intervalo en milisegundos

# Display the graph
try:
    plt.show()
except KeyboardInterrupt:
    rospy.loginfo("Shutting down ROS node...")
    rospy.signal_shutdown("KeyboardInterrupt")


