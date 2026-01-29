#!/usr/bin/env python3

import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from std_msgs.msg import Float32
import numpy as np
from numpy.typing import NDArray
from typing import Tuple


class LineFindingNode:
    def __init__(self):
        # Publishers
        self.img_pub = rospy.Publisher(
            '/line_debug_img', Image, queue_size=10
        )
        self.control = rospy.Publisher(
            '/control', Float32, queue_size=10
        )

        # Subscribers
        rospy.Subscriber('/rrbot/camera1/image_raw', Image, self.process_image)

        # Constants
        self.red_channel_threshold: int = 80
        self.circle_colour: Tuple[int, int, int] = (0, 0, 255) # red
        self.circle_radius: int = 20
        self.x: int = 0
        self.y: int = 0

 
    def get_x_indices_on_line(self, image: NDArray, row_idx: int) -> np.ndarray:
        """
        Produces an array which has the x-index for pixels on the line,
        and zero otherwise
        """

        sample_row = image[row_idx]  # Dimension is (cols, rows)
        red_channel = sample_row[:, 2]  # Note: OpenCV uses BGR

        x_indices = np.indices(red_channel.shape)[0]
        indices_on_line = x_indices[red_channel < self.red_channel_threshold]

        return indices_on_line
    
    def find_line(self, image: NDArray) -> Tuple[int, int]:
        """Locate the line in a frame from raw_video_feed.mp4

        Aims to find a point on the line at a consistent height in the image,
        near the bottom of the frame.

        Args:
            image: Frame in which to find a point on the line

        Returns:
            (x, y) coordinates of a point on the line

        Raises:
            ValueError: If the line is not found in the bottom 150 image rows
        """
        image_height: int = image.shape[0]
        sample_row_idx: int = image_height - 150

        indices_on_line = self.get_x_indices_on_line(image, sample_row_idx)

        # If the line is not found, move down in the image
        while len(indices_on_line) == 0:
            sample_row_idx += 1
            indices_on_line = self.get_x_indices_on_line(image, sample_row_idx)
            if sample_row_idx == (image_height - 1):
                raise ValueError("Line not found in image!")

        x: int = int(indices_on_line.mean())
        y: int = sample_row_idx

        return x, y

    def process_image(self, img: Image):

        rospy.loginfo(f"Received Image!")

        # Used to convert between ROS and OpenCV images
        bridge = CvBridge()
        
        # Convert ROS Image message to OpenCV image
        frame = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")

        # Find the line
        try:
            x, y = self.find_line(frame)
            self.x = x
            self.y = y
            rospy.loginfo(f"Track found at coords: ({self.x}, {self.y})")
        except ValueError:
            rospy.loginfo(f"Track found, using previous coords: ({self.x}, {self.y})")

        # Draw a dot; mutates the image frame
        cv2.circle(frame, (self.x, self.y), self.circle_radius, self.circle_colour, thickness=cv2.FILLED)

        # TODO: determine width from the topic instead of checking each time
        width: int = frame.shape[1]
        
        # Control should be between [-0.5, 0.5] and represents the position of the dot on the screen.
        control: float = (self.x / width) - 0.5
        rospy.loginfo(f"Control signal: {control:.3f}")

        # Convert back to ROS image format
        ros_img = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        self.img_pub.publish(ros_img)
        self.control.publish(Float32(control))

if __name__ == '__main__':
    rospy.init_node('find_line')
    node = LineFindingNode()
    rospy.spin()