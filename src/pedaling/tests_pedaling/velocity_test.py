#!/usr/bin/env python
from __future__ import print_function

# roslaunch kindyn robot.launch robot_name:=rikshaw start_controllers:='joint_hip_left joint_hip_right joint_wheel_right joint_wheel_back joint_pedal spine_joint joint_wheel_left joint_front joint_pedal_right joint_pedal_left elbow_right_rot1 joint_foot_left joint_knee_right joint_knee_left joint_foot_right left_shoulder_axis0 left_shoulder_axis1 left_shoulder_axis2 elbow_left_rot1 elbow_left_rot0 left_wrist_0 left_wrist_1 right_shoulder_axis0 right_shoulder_axis2 right_shoulder_axis1 elbow_right_rot0 right_wrist_0 right_wrist_1 head_axis0 head_axis1 head_axis2'


import math
import time
from threading import Thread

import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy
from roboy_middleware_msgs.srv import InverseKinematics, ForwardKinematics
from std_msgs.msg import Float32

TIME_STEP_SIMULATION = 0.5
TIME_STEP_REALITY = 0.5
VELOCITY_STEPS_REALITY = [2, 5, 10]
ERROR_TOLERANCE_REALITY = 2*np.pi / 720


PEDAL_CENTER_OFFSET_X = 0.20421
PEDAL_CENTER_OFFSET_Y = -0.00062
PEDAL_CENTER_OFFSET_Z = 0.2101

RADIUS_BACK_TIRE = 0.294398  # in m
RADIUS_GEAR_CLUSTER = 0.06  # in m
RADIUS_FRONT_CHAIN_RING = 0.075

ROS_JOINT_HIP_RIGHT = "joint_hip_right"
ROS_JOINT_KNEE_RIGHT = "joint_knee_right"
ROS_JOINT_ANKLE_RIGHT = "joint_foot_right"
ROS_JOINT_HIP_LEFT = "joint_hip_left"
ROS_JOINT_KNEE_LEFT = "joint_knee_left"
ROS_JOINT_ANKLE_LEFT = "joint_foot_left"

RIGHT_HIP_JOINT = "right_hip"
RIGHT_KNEE_JOINT = "right_knee"
RIGHT_ANKLE_JOINT = "right_ankle"
LEFT_HIP_JOINT = "left_hip"
LEFT_KNEE_JOINT = "left_knee"
LEFT_ANKLE_JOINT = "left_ankle"

joint_status_data = {
    RIGHT_HIP_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    LEFT_HIP_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    RIGHT_KNEE_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    LEFT_KNEE_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    RIGHT_ANKLE_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    LEFT_ANKLE_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    }
}


def getPositionLeftFoot():
    fkJointNamesList = [ ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_LEFT, ROS_JOINT_ANKLE_LEFT ]
    fkJointPositions = [ joint_status_data[ LEFT_HIP_JOINT ][ "Pos" ], joint_status_data[ LEFT_KNEE_JOINT ][ "Pos" ],
                         joint_status_data[ LEFT_ANKLE_JOINT ][ "Pos" ] ]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("foot_left_tip", "foot_left_tip", fkJointNamesList, fkJointPositions)
        return [ fk_result.pose.position.x, fk_result.pose.position.z ]

    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

    print("ERROR fk foot_left failed")
    return [ 0.0, 0.0 ]  # [x, z]


def getPositionRightFoot():
    fk_joint_names_list = [ ROS_JOINT_HIP_RIGHT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_ANKLE_RIGHT ]
    fk_joint_positions = [ joint_status_data[ RIGHT_HIP_JOINT ][ "Pos" ],
                           joint_status_data[ RIGHT_KNEE_JOINT ][ "Pos" ],
                           joint_status_data[ RIGHT_ANKLE_JOINT ][ "Pos" ] ]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("foot_right_tip", "foot_right_tip", fkJointNamesList, fkJointPositions)
        return [ fk_result.pose.position.x, fk_result.pose.position.z ]

    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

    print("ERROR fk foot_right failed")
    return [ 0.0, 0.0 ]  # [x, z]


def evaluate_current_angle(current_point):
    current_x = current_point[ 0 ] - PEDAL_CENTER_OFFSET_X
    current_y = current_point[ 1 ] - PEDAL_CENTER_OFFSET_Y

    if current_x > 0 and current_y > 0:
        return np.arctan(current_y / current_x)
    elif current_x < 0 and current_y > 0:
        return np.arctan(current_y / current_x) + np.pi
    elif current_x < 0 and current_y < 0:
        return np.arctan(current_y / current_x) + np.pi
    elif current_x > 0 and current_y < 0:
        return np.arctan(current_y / current_x) + 2 * np.pi

    elif current_x == 0 and current_y > 0:
        return np.pi / 2
    elif current_x == 0 and current_y < 0:
        return np.pi * 3 / 2
    elif current_x > 0 and current_y == 0:
        return 0
    elif current_x < 0 and current_y == 0:
        return np.pi


def get_angle_difference(angle_1, angle_2):
    return np.pi - np.abs(np.abs(angle_1 - angle_2) - np.pi)


###### Test-Methods #####


def evaluate_error(velocity, leg):

    current_angle = 0
    next_angle = 0
    if leg == "right":
        current_angle = evaluate_current_angle(getPositionRightFoot())
        time.sleep(TIME_STEP_SIMULATION)
        next_angle = evaluate_current_angle(getPositionRightFoot())
    elif leg == "left":
        current_angle = evaluate_current_angle(getPositionLeftFoot())
        time.sleep(TIME_STEP_SIMULATION)
        next_angle = evaluate_current_angle(getPositionLeftFoot())

    circulation_time = 2 * np.pi * (RADIUS_FRONT_CHAIN_RING / RADIUS_GEAR_CLUSTER /
                                                          (velocity / RADIUS_BACK_TIRE))
    target_angle = (current_angle + (np.pi * 2 / circulation_time * TIME_STEP_SIMULATION)) % (2 * np.pi)

    return get_angle_difference(target_angle, next_angle)


def simulation_test(pub):

    error_results_right = [[]] * 20
    error_results_left = [[]] * 20

    for i in range(1, 21):
        pub.publish(i)
        for k in range(10):
            error_results_right[i].append(evaluate_error(i, "right"))
            error_results_left[i].append(evaluate_error(i, "left"))

    avg_error_left = []
    avg_error_right = []
    max_error_left = []
    max_error_right = []
    for velocity in error_results_right:
        avg_error_right.append(sum(velocity) / len(velocity))
        max_error_right.append(max(velocity))
    for velocity in error_results_left:
        avg_error_left.append(sum(velocity) / len(velocity))
        max_error_left.append(max(velocity))

    plt.plot(range(1, 21), avg_error_left, label="average error left")
    plt.plot(range(1, 21), avg_error_right, label="average error right")
    plt.plot(range(1, 21), max_error_left, label="max error left")
    plt.plot(range(1, 21), max_error_right, label="max error right")
    plt.ylabel("error value")
    plt.xlabel("velocity")
    plt.show()


def velocity_reached(velocity):
    current_angle = evaluate_current_angle(getPositionRightFoot())
    time.sleep(TIME_STEP_REALITY)
    next_angle = evaluate_current_angle(getPositionRightFoot())

    circulation_time = 2 * np.pi * (RADIUS_FRONT_CHAIN_RING / RADIUS_GEAR_CLUSTER /
                                    (velocity / RADIUS_BACK_TIRE))
    target_angle = (current_angle + (np.pi * 2 / circulation_time * TIME_STEP_REALITY)) % (2 * np.pi)

    angle_error = get_angle_difference(next_angle, target_angle)

    return angle_error < ERROR_TOLERANCE_REALITY


def reality_test_acceleration(pub):
    acceleration_times = [[]] * len(VELOCITY_STEPS_REALITY)

    for j in range(len(VELOCITY_STEPS_REALITY)):
        for i in range(1, 21, VELOCITY_STEPS_REALITY[i]):
            start_time = rospy.get_rostime()
            pub.publish(i)
            while not velocity_reached(i):
                pass
            end_time = rospy.get_rostime()
            acceleration_time = rospy.Duration(end_time-start_time).to_sec()
            acceleration_times[j].append(acceleration_time)

    for i in range(len(VELOCITY_STEPS_REALITY)):
        plt.plot(range(1, 21, VELOCITY_STEPS_REALITY[i]), acceleration_times[i],
                 label="velocity steps = "+str(VELOCITY_STEPS_REALITY))

    plt.xlabel("velocity")
    plt.ylabel("acceleration time in s")
    plt.show()



def main():
    pub = rospy.Publisher('/cmd_velocity_rickshaw', Float32, queue_size=10)
    rospy.init_node('velocity_publisher', anonymous=True)
    simulation_test(pub)
    reality_test_acceleration(pub)
    return 1


if __name__ == '__main__':
    main()