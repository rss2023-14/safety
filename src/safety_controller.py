#!/usr/bin/env python2

import numpy as np
import message_filters

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *
import math


def safety_control(ackermann, lidar_data, pub):
    if lidar_data.angle_increment == 0.0:
        return  # Early return for uninitialized LaserScan

    # how far in seconds do we want to check
    look_ahead_time = rospy.get_param("safety_controller/look_ahead_time")
    half_car_width = rospy.get_param("safety_controller/half_car_width")

    # we only want from -90 deg to 90 deg
    low_index = int(
        (-(math.pi/2.0) - lidar_data.angle_min)/lidar_data.angle_increment)
    high_index = int(
        ((math.pi/2.0) - lidar_data.angle_min)/lidar_data.angle_increment)

    ranges = np.array(lidar_data.ranges)[low_index:high_index]
    angles = np.arange(lidar_data.angle_min,
                       lidar_data.angle_max, lidar_data.angle_increment)[low_index:high_index]

    # if we are turning left we have to multiply by -1 in places
    # find the radius the car is turning around
    if ackermann.drive.steering_angle == 0.0:
        turning_right = 1.0
        turning_radius = 1000.0
    elif ackermann.drive.steering_angle < 0.0:
        turning_right = 1.0
        turning_radius = abs(
            0.35 / np.tan(ackermann.drive.steering_angle))
    else:
        turning_right = -1.0
        turning_radius = abs(
            0.35 / np.tan(ackermann.drive.steering_angle))

    max_angle = (look_ahead_time * ackermann.drive.speed) / turning_radius

    # take the coordinates from the lidar's perspective and find what they are from the turning radius point
    # the angle is relative to our turning radius point to the lidar, positive if ahead of the car
    # then see if there are any points in the car's path, and if so stop the car
    for index, range in enumerate(ranges):
        if range < 0.10:
            continue

        inner_angle = (math.pi/2.0) - (turning_right * angles[index])

        dist = math.sqrt(range**2.0 + turning_radius**2.0 -
                         (2.0 * range * turning_radius * np.cos(inner_angle)))

        angle = np.arcsin((range/dist) * np.sin(inner_angle))

        if dist > turning_radius - half_car_width and dist < turning_radius + half_car_width and angle < max_angle:
            rospy.loginfo("Safety Controller engaged!")

            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "base_link"

            msg.drive.speed = 0

            msg.drive.steering_angle_velocity = 0.0
            msg.drive.acceleration = 0.0
            msg.drive.jerk = 0.0

            pub.publish(msg)
            break


if __name__ == "__main__":
    rospy.init_node('safety_controller')
    rospy.Rate(20)  # 20hz

    ackermann_sub = message_filters.Subscriber(
        "/vesc/high_level/ackermann_cmd_mux/output", AckermannDriveStamped)
    lidar_sub = message_filters.Subscriber("/scan", LaserScan)

    pub = rospy.Publisher(
        "/vesc/low_level/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=1)

    ts = message_filters.ApproximateTimeSynchronizer(
        [ackermann_sub, lidar_sub], 10, 0.05)
    ts.registerCallback(safety_control, pub)

    rospy.spin()
