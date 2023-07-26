#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

class SquareMovementNode:
    def __init__(self):
        rospy.init_node('square_movement_node', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/curiosity_mars_rover/ackermann_drive_controller/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.linear_speed = 1.0  # meters per second (adjust as needed)
        self.angular_speed = math.pi/2  # radians per second (90 degrees)

    def move_in_square(self):
        # Create a Twist message and set the linear and angular velocities
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0

        # Move forward for the side length of the square
        side_length = 1.0  # meters (adjust as needed)
        duration = rospy.Duration.from_sec(side_length / self.linear_speed)
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < duration:
            self.velocity_publisher.publish(twist)
            self.rate.sleep()

        # Stop for 1 second
        twist.linear.x = 0.0
        self.velocity_publisher.publish(twist)
        rospy.sleep(1)

        # Rotate 90 degrees
        twist.angular.z = self.angular_speed
        duration = rospy.Duration.from_sec((math.pi / 2) / self.angular_speed)
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < duration:
            self.velocity_publisher.publish(twist)
            self.rate.sleep()

        # Stop for 1 second
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)
        rospy.sleep(1)

    def run(self):
        while not rospy.is_shutdown():
            self.move_in_square()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = SquareMovementNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

