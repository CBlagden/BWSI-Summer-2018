import rospy
import tf
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from controllers import PIDController

NODE_NAME = 'drive_forward'


class DriveForwardNode:

    def __init__(self):
        self.drive_publisher = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0',
                                               AckermannDriveStamped, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/vesc/odom', Odometry, self.odom_callback,
                                                queue_size=10)

        self.drive_controller = PIDController(kP=1)
        self.steer_controller = PIDController(kP=0.01)
        self.drive_msg = AckermannDriveStamped()

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        #steer_output = self.steer_controller.output(yaw, 0)
        steer_output = -1
        drive_output = self.drive_controller.output(x, 0)
        print("Error: %s" % (x - 0))
        self.drive_msg.drive.steering_angle = steer_output
        self.drive_msg.drive.speed = drive_output
        self.drive_publisher.publish(self.drive_msg)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    d = DriveForwardNode()
    rospy.spin()
