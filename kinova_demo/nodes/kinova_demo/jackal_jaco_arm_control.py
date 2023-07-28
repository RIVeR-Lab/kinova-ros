#! /usr/bin/python3
import rospy

import sys

import math

import actionlib
import kinova_msgs.msg

import argparse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

""" Global variable """
arm_joint_number = 6
finger_number = 3
prefix = 'j2n6s300_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentJointCommand = [] # number of joints is defined in __main__


def joint_angle_client(angle_set):
    """Send a joint angle goal to the action server."""
    action_address = '/' + prefix + 'driver/joints_action/joint_angles'
    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.ArmJointAnglesAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmJointAnglesGoal()

    goal.angles.joint1 = angle_set[0]
    goal.angles.joint2 = angle_set[1]
    goal.angles.joint3 = angle_set[2]
    goal.angles.joint4 = angle_set[3]
    goal.angles.joint5 = angle_set[4]
    goal.angles.joint6 = angle_set[5]
    goal.angles.joint7 = angle_set[6]

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(20.0)):
        return client.get_result()
    else:
        print('        the joint angle action timed-out')
        client.cancel_all_goals()
        return None



def getcurrentJointCommand(prefix_):
    # wait to get current position
    topic_address = '/' + prefix_ + 'driver/out/joint_command'
    rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, setcurrentJointCommand)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.JointAngles)
    print('position listener obtained message for joint position. ')


def setcurrentJointCommand(feedback):
    global currentJointCommand

    currentJointCommand_str_list = str(feedback).split("\n")
    for index in range(0,len(currentJointCommand_str_list)):
        temp_str=currentJointCommand_str_list[index].split(": ")
        currentJointCommand[index] = float(temp_str[1])



def unitParser(unit, joint_value, relative_):
    """ Argument unit """
    global currentJointCommand

    if unit == 'degree':
        joint_degree_command = joint_value
        # get absolute value
        if relative_:
            joint_degree_absolute_ = [joint_degree_command[i] + currentJointCommand[i] for i in range(0, len(joint_value))]
        else:
            joint_degree_absolute_ = joint_degree_command
        joint_degree = joint_degree_absolute_
        joint_radian = list(map(math.radians, joint_degree_absolute_))
    elif unit == 'radian':
        joint_degree_command = list(map(math.degrees, joint_value))
        # get absolute value
        if relative_:
            joint_degree_absolute_ = [joint_degree_command[i] + currentJointCommand[i] for i in range(0, len(joint_value))]
        else:
            joint_degree_absolute_ = joint_degree_command
        joint_degree = joint_degree_absolute_
        joint_radian = list(map(math.radians, joint_degree_absolute_))
    else:
        raise Exception("Joint value have to be in degree, or radian")

    return joint_degree, joint_radian


def trajectory_callback(msg):
    print("[+] Trajectory Received")
    # for point in msg.points[0]:
    try:
        for idx, point in enumerate(msg.points):
            if idx != 0 or idx != len(msg.points - 1):
                continue
            joint_degree, joint_radian = unitParser('radian', point.positions, False)
            positions = [0]*7
            if arm_joint_number < 1:
                print('Joint number is 0, check with "-h" to see how to use this node.')
                positions = []  # Get rid of static analysis warning that doesn't see the exit()
                sys.exit() 
            else:
                for i in range(0,arm_joint_number):
                    positions[i] = joint_degree[i]               

            result = joint_angle_client(positions)

    except rospy.ROSInterruptException:
        print('program interrupted before completion')




if __name__ == '__main__':


    rospy.init_node(prefix + 'gripper_workout')

    # currentJointCommand = [0]*arm_joint_number
    # KinovaType defines AngularInfo has 7DOF, so for published topics on joints.
    currentJointCommand = [0]*7

    # get Current finger position if relative position
    getcurrentJointCommand(prefix)
    print("[+] Node Started")
    trajectory_subscriber = rospy.Subscriber('/j2n6s300_driver/trajectory_controller/command', JointTrajectory, queue_size=100, callback=trajectory_callback)
    # joint_degree, joint_radian = unitParser('radian', '')
    rospy.spin()
