#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class LineFollowingNode:
    def __init__(self):

        # Publishers
        self.cmd_pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=1
        )

        # Subscribers
        rospy.Subscriber('/control', Float32, self.get_twist)

        self.proportional: float = -20.0
        self.derivative: float = -30.0
        self.d_control: float = 0
        self.prev_control: float | None = None

    def get_twist(self, control: Float32) -> None:
        move: Twist = Twist()


        if self.prev_control is None:
            self.prev_control = control.data

        # Derivative with exponential smoothing
        alpha = 0.8
        self.d_control = alpha * (control.data - self.prev_control) + (1 - alpha) * self.d_control

        # Adjust control based on how far we are on the line
        # distance_to_line_normalized =  2 * abs(control.data)

        move.linear.x = 5.0

        move.angular.z = self.proportional * control.data + self.derivative * (control.data - self.prev_control)

        
        

        self.prev_control = control.data

        rospy.loginfo(f"{move.linear.x=}")
        rospy.loginfo(f"{move.angular.z=}")
        
        self.cmd_pub.publish(move)

if __name__ == '__main__':
    rospy.init_node('line_follow')
    node = LineFollowingNode()
    rospy.spin()
