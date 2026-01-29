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

        self.proportional: float = -19.0

    def get_twist(self, control: Float32) -> None:
        move: Twist = Twist()
        move.linear.x = 5.0
        move.angular.z = self.proportional * control.data

        rospy.loginfo(f"{move.linear.x=}")
        rospy.loginfo(f"{move.angular.z=}")
        
        self.cmd_pub.publish(move)

if __name__ == '__main__':
    rospy.init_node('line_follow')
    node = LineFollowingNode()
    rospy.spin()
