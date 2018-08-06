#!/usr/bin/env python
"""
Rahul, Chase, William BWSI 2018 Team08
"""
import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

from controllers import PIDController


class WallFollowerNode:

    def __init__(self):

        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan,
                                                 self.laser_callback, queue_size=10)
        self.drive_publisher = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped,
                                               queue_size=10)

        self.steering_controller = PIDController(kP=2.0, kD=0)

        self.drive_msg = AckermannDriveStamped()

        self.goal_distance = 0.8
<<<<<<< HEAD
        self.drive_speed = 1.0
=======
        self.drive_speed = 1.2
>>>>>>> 6ca5c89d31c5c5574427ff803e29943e28983a42

    def laser_callback(self, msg):

        cur_dist = min(msg.ranges[int(0.4*len(msg.ranges)):int(0.85*len(msg.ranges))])
        front_dist = msg.ranges[len(msg.ranges)/2]

        steering_output = self.steering_controller.output(cur_dist, self.goal_distance)

        if front_dist < 0.8:
            self.drive_speed = 0.7
            print("Speed @ 0.7")
        else:
            self.drive_speed = 1.2
            print("Speed @ 1.2")

        self.drive_msg.drive.steering_angle = -steering_output if  cur_dist < 1.1 else 100.0
        self.drive_msg.drive.speed = self.drive_speed
        print("Distance from wall: %s" % cur_dist)                
        print("Error: %s" % (cur_dist - self.goal_distance))
        self.drive_publisher.publish(self.drive_msg)

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    s = WallFollowerNode()
    rospy.spin()
