import rospy
import tf

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from constants import *
from controllers import PurePursuitController, Vector, Pose


class PurePursuitTestNode:

    def __init__(self):
        rospy.init_node('pure_pursuit_node')

        self.drive_pub = rospy.Publisher(DRIVE_PUBLISHER_TOPIC, AckermannDriveStamped,
                                         queue_size=10)
        self.odom_sub = rospy.Subscriber(ODOM_SUBSCRIBER_TOPIC, Odometry,
                                         self.odom_callback, queue_size=10)
        self.drive_msg = AckermannDriveStamped()

        waypoints = [Vector(0, 1)]
        self.pure_pursuit_controller = PurePursuitController(waypoints, lookahead_dist=0.2)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        _, _, yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        x, y = p.x, p.y

        print("X: %s" % x)
        print("Y: %s" % y)
        print("Yaw: %s" % yaw)           

        angle_output = -self.pure_pursuit_controller.update(Pose(x, y, yaw))

        print("Angular output: %s" % angle_output)        

        self.drive_msg.drive.steering_angle = angle_output
        self.drive_msg.drive.speed = 0.7

        self.drive_pub.publish(self.drive_msg)


if __name__ == '__main__':
    p = PurePursuitTestNode()
    rospy.spin()
