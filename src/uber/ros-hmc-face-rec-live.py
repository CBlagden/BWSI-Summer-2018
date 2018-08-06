#!/usr/bin/env python
#Nam Tran HMC 2018
#Adapted for ROS by Mayank Mali

# import the necessary packages
import face_recognition
import argparse
import imutils
import pickle
import time
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# TO RUN: python faceRec_cam.py --encodings encodings.pickle

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser("A ros wrapper for the HMC face recognition to use with ros zed wrapper")
ap.add_argument("-e", "--encodings", required=True,
	help="path to serialized db of facial encodings")
ap.add_argument("-d", "--detection-method", type=str, default="cnn",
	help="face detection model to use: either `hog` or `cnn`")
ap.add_argument("-s", "--subscribe-topic", dest="subscribe_topic", type=str, required=True, help="ros topic to subscribe.")

ap.add_argument("-p", "--publish-topic", dest="publish_topic", type=str, required=True, help="ros topic to publish.")

args = vars(ap.parse_args())

# load the known faces and embeddings
print("[INFO] loading encodings...")
data = pickle.loads(open(args["encodings"], "rb").read())

# initialize the video stream and pointer to output video file, then
# allow the camera sensor to warm up
print("[INFO] starting video stream...")

# reads in source for cam feed
# if VideoCapture(0) doesn't work, try -1, 1, 2, 3 (if none of those work, the webcam's not supported!)
#cam = cv2.VideoCapture(0)

rospy.init_node("hmc_face_detector")

img_placeholder = np.array([-1])
bridge = CvBridge()

def callback(imgmsg):
    global img_placeholder
    img_placeholder = bridge.imgmsg_to_cv2(imgmsg)

rospy.Subscriber(args["subscribe_topic"], Image, callback)

pub = rospy.Publisher(args["publish_topic"], Image, queue_size=1)

while(img_placeholder.size == 1):
	pass

rate = rospy.Rate(20)
# loop over frames from the video stream
try:
    while True:
        # grab the frame from the video stream
        #ret_val, frame = cam.read()
        frame = img_placeholder.copy()

        if(True):
            # convert the input frame from BGR to RGB then resize it to have
            # a width of 750px (to speedup processing)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb = imutils.resize(frame, width=750)
            r = frame.shape[1] / float(rgb.shape[1])

            # detect the (x, y)-coordinates of the bounding boxes
            # corresponding to each face in the input frame, then compute
            # the facial embeddings for each face
            boxes = face_recognition.face_locations(rgb, model=args["detection_method"])
            encodings = face_recognition.face_encodings(rgb, boxes)
            names = []
            label=""
		
            # loop over the facial embeddings
            for encoding in encodings:
                # attempt to match each face in the input image to our known encodings
                matches = face_recognition.compare_faces(data["encodings"],
                        encoding)
                name = "Threat!!!!!"

                # check to see if we have found a match
                if True in matches:
                    # find the indexes of all matched faces then initialize a
                    # dictionary to count the total number of times each face
                    # was matched
                    matchedIdxs = [i for (i, b) in enumerate(matches) if b]
                    counts = {}

                    # loop over the matched indexes and maintain a count for
                    # each recognized face
                    for i in matchedIdxs:
                            name = data["names"][i]
                            counts[name] = counts.get(name, 0) + 1

                    # determine the recognized face with the largest number
                    # of votes (note: in the event of an unlikely tie Python
                    # will select first entry in the dictionary)
                    name = max(counts, key=counts.get)

                    # stores the numpy array of Euclidean distances 
                    predictions = face_recognition.face_distance(data["encodings"], encoding)
                    # loops through the numpy array and converts the floats to percentages for labeling
                    for i, percent in enumerate(predictions):
                        # print ((": {:.2%}".format(percent, i))[:7] + '%')
                        label = (": {:.2%}".format(percent, i))[:7] + '%'
            
                # update the list of names
                names.append(name)

            # loop over the recognized faces
            for ((top, right, bottom, left), name) in zip(boxes, names):
                # rescale the face coordinates
                top = int(top * r)
                right = int(right * r)
                bottom = int(bottom * r)
                left = int(left * r)

                # draw the predicted face name on the image
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
                
                y = top - 15 if top - 15 > 15 else top + 15

                cv2.putText(frame, name + label, (left, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

            pub.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            rospy.loginfo("running")
            #cv2.imshow("Frame", frame)
        rate.sleep()    
        #if(cv2.waitKey(1) & 0xFF == ord("q")):
        #    break
except rospy.ROSInterruptException:
	exit()	
except KeyboardInterrupt:
	exit()

# cleanup script
