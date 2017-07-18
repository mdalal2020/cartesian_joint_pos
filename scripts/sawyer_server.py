#!/usr/bin/env python

from cartesian_joint_pos.srv import *
import rospy
import numpy as np
from urdf_parser_py.urdf import URDF 
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
import intera_interface as ii
from pykdl_utils.kdl_kinematics import KDLKinematics

def handle_jointCartesianPos(req):
    arm = ii.Limb(req.name)
    q = arm.joint_angles()
    q = [q['right_j0'], q['right_j1'], q['right_j2'], q['right_j3'], q['right_j4'], q['right_j5'], q['right_j6']]
    pose = kin.forward(q, end_link=req.name + req.tip)
    pose = np.squeeze(np.asarray(pose))
    pose = [pose[0][3], pose[1][3], pose[2][3]]
    return getCartesianJointPosResponse(pose)

def jointCartesianPos_server():
    rospy.init_node('jointCartesianPos_server')
    global kin
    robot = URDF.from_parameter_server(key='robot_description')
    base_link = 'base'
    end_link = 'right_hand'
    kin = KDLKinematics(robot, base_link, end_link)
    s = rospy.Service('getCartesianJointPos', getCartesianJointPos, handle_jointCartesianPos)
    rospy.spin()

if __name__ == "__main__":
    jointCartesianPos_server()
