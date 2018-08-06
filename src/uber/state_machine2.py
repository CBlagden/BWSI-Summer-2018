#!/usr/bin/env python
"""
Chase, Rahul, William BWSI 2018 Team08
"""
import os
import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

import cv2
import face_recognition
from controllers import PIDController

STATE_MACHINE_NODE_NAME = 'state_machine'
IMAGES_DIR = 'images'

faces_encodings = [None] * 8
ta_tag_dict = {
    'andrew.png': 1,
    'leo.png': 2,
    'mayank.png': 3,
    'karen.png': 4,
    'maryam.png': 5,
    'syed.png': 6,
    'carlos.png': 7,
    'bryan.png': 8
}

class State:
	search_face = 0
	approach_face = 1
	search_tag = 2
	approach_tag = 3


def init():
    for filename in os.listdir(IMAGES_DIR):
        img = face_recognition.load_image_file(IMAGES_DIR + '/' + filename)
        face_encoding = face_recognition.face_encodings(img)
        faces_encodings[ta_tag_dict[filename]-1] = face_encoding

class StateMachineNode:
    def __init__(self):
        rospy.init_node(STATE_MACHINE_NODE_NAME)

        self.drive_publisher = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=10)

        self.ar_subscriber = rospy.Subscriber('/ar_pose_marker', AlvarMarkers,
                                              self.ar_callback, queue_size=10)
        print "Ar subscriber created"
        self.img_subscriber = rospy.Subscriber('/zed/rgb/image_rect_color', Image,
                                               self.img_callback, queue_size=10)
        print "Img subscriber created"
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan,
                                                 self.laser_callback, queue_size=10)
  #      print "Laser subscriber created"
 #       self.drive_publisher = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped,
#                                               queue_size=10)
        print "Drive publisher created"

        self.steering_controller = PIDController(kP=0.1)

        self.drive_msg = AckermannDriveStamped()
        self.cv_bridge = CvBridge()

        self.goal_tag = -1
        self.goal_distance = 0.7
        self.drive_speed = 0.7
        self.ar_tag_threshold = 0.8
    
        self.state = State.search_face

        self.haar = cv2.CascadeClassifier('./haarcascade_frontalface_default.xml')

    def ar_callback(self, msg):
		if len(msg.markers) > 0:
			if self.goal_tag in msg.markers and self.state == State.search_tag:
				tagdata = [x for x in msg.markers if x.id == self.goal_tag][0]
				if tagdata.pose.pose.position < self.ar_tag_threshold: return
				self.state = State.approach_tag
				self.drive_speed = 0.7
			elif self.goal_tag not in msg.markers and self.state == State.approach_tag:
				self.state = State.search_face	

    def img_callback(self, msg):
        global faces_encodings

        img_cv = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        rows, cols = img_cv.shape[0], img_cv.shape[1]
        # Find face in image
        faces = self.haar.detectMultiScale(img_cv, scaleFactor=1.1, minNeighbors=5)
        if self.state == State.search_face:
            for x, y, w, h in faces:
                # Get encoding
                encoding = face_recognition.face_encodings(img_cv[x:x+w+1:,y:y+h+1:, :], known_face_locations=None)
                # Compare the encoding to the list of faces
                results = face_recognition.api.compare_faces(faces_encodings, encoding)
                if any(results):
                    self.goal_tag = min([i for i in range(len(results)) if results[i] == True])
                    self.state = State._face
        elif self.state == State.approach_face and len(faces) == 0:
            self.state = State.search_tag
            self.drive_speed = -0.7
        

    def laser_callback(self, msg):
        # If it's searching for a face or a tag, stay away from walls
        if self.state == State.search_face or self.state == State.search_tag:
            self.goal_distance = 0.7
        # If it's found something, approach it
        elif self.state == State.approach_face or self.state == State.approach_tag:
            self.goal_distance = 0.2

        cur_dist = min(msg.ranges[int(0.5*len(msg.ranges)):])
        output = self.steering_controller.output(cur_dist, self.goal_distance)

        self.drive_msg.drive.steering_angle = output
        self.drive_msg.drive.speed = self.drive_speed

        self.drive_publisher.publish(self.drive_msg)


if __name__ == '__main__':
    rospy.node_init('state_machine')
    s = StateMachineNode()

