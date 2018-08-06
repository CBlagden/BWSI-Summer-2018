import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
from ar_track_alvar_msgs.msg import AlvarMarkers
from controllers import PIDController
import json


class State:
    search = 0
    approach = 1


class StateMachineNode:

    def __init__(self):
        self.drive_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",
                                         AckermannDriveStamped, queue_size=10)
        self.line_sub = rospy.Subscriber('/line_error', Float32, self.line_callback, queue_size=10)
        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback, queue_size=10)

        self.msg = AckermannDriveStamped()
        self.steering_controller = PIDController(kP=0.0007)

        self.state = State.search
        self.prev_angle = 0
        self.ar_markers = []

    def line_callback(self, msg):
        output = 0
        if len(self.ar_markers) > 0:
            for i in self.ar_markers:
                if i.id == 3 and i.pose.pose.position.z < 0.55:
                    print(i.pose.pose.position.z)
                    self.msg.drive.speed = 0
        else:    
            output = self.steering_controller.output_error(msg.data)
            self.msg.drive.steering_angle = -output
           # speed = abs(0.05 / (output + 0.001) + 0.5)
            speed = 1.0
            #print("Speed: %s" % speed)
            self.msg.drive.speed = speed
        self.drive_pub.publish(self.msg)
        self.prev_angle = output

    def ar_callback(self, msg):
        self.ar_markers = msg.markers


if __name__ == '__main__':
    rospy.init_node("state_machine", anonymous=True)
    n = StateMachineNode()
    rospy.spin()

