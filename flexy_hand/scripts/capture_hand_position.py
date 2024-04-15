#!/usr/bin/env python3

import rospy
from flexy_hand.msg import FingersAngles
import cv2 as cv
import mediapipe as mp
import numpy as np
from utils.utils_handpose3d import DLT, get_projection_matrix 
from utils.utils_calcangles import angle3points

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

frame_shape = [720, 1280]

def run_mp(input_stream1, input_stream2, P0, P1):

    # ROS Node
    pub = rospy.Publisher('fingers_angles', FingersAngles, queue_size=10)
    rospy.init_node('hand_detector', anonymous=False)

    # Input video stream
    cap0 = cv.VideoCapture(input_stream1)
    cap1 = cv.VideoCapture(input_stream2)
    caps = [cap0, cap1]

    # Set camera resolution if using webcam to 1280x720. Any bigger will cause some lag for hand detection
    for cap in caps:
        cap.set(3, frame_shape[1])
        cap.set(4, frame_shape[0])

    # Create hand keypoints detector object.
    hands0 = mp_hands.Hands(min_detection_confidence=0.5, max_num_hands =1, min_tracking_confidence=0.5)
    hands1 = mp_hands.Hands(min_detection_confidence=0.5, max_num_hands =1, min_tracking_confidence=0.5)

    while not rospy.is_shutdown():
        # Read frames from stream
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()

        if not ret0 or not ret1: break
        
        # BGR image to RGB
        frame0 = cv.cvtColor(frame0, cv.COLOR_BGR2RGB)
        frame1 = cv.cvtColor(frame1, cv.COLOR_BGR2RGB)

        # To improve performance, optionally mark the image as not writeable to pass by reference.
        frame0.flags.writeable = False
        frame1.flags.writeable = False
        results0 = hands0.process(frame0)
        results1 = hands1.process(frame1)

        # Prepare list of hand keypoints of this frame
        
        # frame0 kpts
        frame0_keypoints = []
        if results0.multi_hand_landmarks:
            for hand_landmarks in results0.multi_hand_landmarks:
                for p in range(21):
                    #print(p, ':', hand_landmarks.landmark[p].x, hand_landmarks.landmark[p].y)
                    pxl_x = int(round(frame0.shape[1]*hand_landmarks.landmark[p].x))
                    pxl_y = int(round(frame0.shape[0]*hand_landmarks.landmark[p].y))
                    kpts = [pxl_x, pxl_y]
                    frame0_keypoints.append(kpts)
        else:
            # If no keypoints are found, simply fill the frame data with [-1,-1] for each kpt
            frame0_keypoints = [[-1, -1]]*21

        # frame1 kpts
        frame1_keypoints = []
        if results1.multi_hand_landmarks:
            for hand_landmarks in results1.multi_hand_landmarks:
                for p in range(21):
                    #print(p, ':', hand_landmarks.landmark[p].x, hand_landmarks.landmark[p].y)
                    pxl_x = int(round(frame1.shape[1]*hand_landmarks.landmark[p].x))
                    pxl_y = int(round(frame1.shape[0]*hand_landmarks.landmark[p].y))
                    kpts = [pxl_x, pxl_y]
                    frame1_keypoints.append(kpts)
        else:
            # If no keypoints are found, simply fill the frame data with [-1,-1] for each kpt
            frame1_keypoints = [[-1, -1]]*21

        # Calculate 3d position
        frame_p3ds = []
        for uv1, uv2 in zip(frame0_keypoints, frame1_keypoints):
            if uv1[0] == -1 or uv2[0] == -1:
                _p3d = [-1, -1, -1]
            else:
                _p3d = DLT(P0, P1, uv1, uv2) # Calculate 3d position of keypoint
            frame_p3ds.append(_p3d)
        

        frame_p3ds = np.array(frame_p3ds).reshape((21, 3))

        
        # Extract 3D landmarks
        #wrist = frame_p3ds[0,  :]
        thumb0 = frame_p3ds[2,  :]
        thumb1 = frame_p3ds[3,  :]
        thumb2 = frame_p3ds[4,  :]
        index0 = frame_p3ds[5,  :]
        index1 = frame_p3ds[6,  :]
        index2 = frame_p3ds[7,  :]
        long0  = frame_p3ds[9,   :]
        long1  = frame_p3ds[10,  :]
        long2  = frame_p3ds[11,  :]
        ring0  = frame_p3ds[13,  :]
        ring1  = frame_p3ds[14,  :]
        ring2  = frame_p3ds[15,  :]
        small0 = frame_p3ds[17, :]
        small1 = frame_p3ds[18, :]
        small2 = frame_p3ds[19, :]

        # Angle calculations
        thumb_angle = angle3points(thumb1, thumb0, thumb2)
        index_angle = angle3points(index1, index0, index2)
        long_angle  = angle3points(long1,  long0,  long2)
        ring_angle  = angle3points(ring1,  ring0,  ring2)
        small_angle = angle3points(small1, small0, small2)

        # Check angles boundaries
        try: thumb_angle = round(np.rad2deg(thumb_angle))
        except: thumb_angle = 180
        try: index_angle = round(np.rad2deg(index_angle))
        except: index_angle = 180
        try: long_angle = round(np.rad2deg(long_angle))
        except: long_angle = 180
        try: ring_angle = round(np.rad2deg(ring_angle))
        except: ring_angle = 180
        try: small_angle = round(np.rad2deg(small_angle))
        except: small_angle = 180

        # Create angles message
        msg = FingersAngles()
        msg.thumb = thumb_angle
        msg.index = index_angle
        msg.long  = long_angle
        msg.ring  = ring_angle
        msg.small = small_angle

        rospy.loginfo(msg)
        
        # Send message
        pub.publish(msg)

        # Draw the hand annotations on the image.
        frame0.flags.writeable = True
        frame1.flags.writeable = True
        frame0 = cv.cvtColor(frame0, cv.COLOR_RGB2BGR)
        frame1 = cv.cvtColor(frame1, cv.COLOR_RGB2BGR)
        if results0.multi_hand_landmarks:
          for hand_landmarks in results0.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame0, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        if results1.multi_hand_landmarks:
          for hand_landmarks in results1.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame1, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        # Display webcam detection
        cv.imshow('cam1', frame1)
        cv.imshow('cam0', frame0)

        k = cv.waitKey(1)
        if k & 0xFF == 27: break # 27 is ESC key.

    cv.destroyAllWindows()
    for cap in caps:
        cap.release()

if __name__ == '__main__':

    # Indicate webcam device numbers
    input_stream1 = 2
    input_stream2 = 0

    try:
        # Projection matrices
        P0 = get_projection_matrix(0)
        P1 = get_projection_matrix(1)

        run_mp(input_stream1, input_stream2, P0, P1)
    except rospy.ROSInterruptException:
        pass