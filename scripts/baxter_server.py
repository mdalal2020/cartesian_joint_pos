#!/usr/bin/env python

from cartesian_joint_pos.srv import *
import rospy
import numpy as np
from urdf_parser_py.urdf import URDF 
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
import baxter_interface as bi
from pykdl_utils.kdl_kinematics import KDLKinematics

def handle_jointCartesianPos(req):
    if req.name == 'right':
        kin = right_kin
    else:
        kin = left_kin
    arm = bi.Limb(req.name)
    q = arm.joint_angles()
    q = [q[req.name + '_s0'], q[req.name + '_s1'], q[req.name + '_e0'], q[req.name + '_e1'], q[req.name + '_w0'], q[req.name + '_w1'], q[req.name + '_w2']]
    pose = kin.forward(q, end_link=req.name + req.tip)
    pose = np.squeeze(np.asarray(pose))
    pose = [pose[0][3], pose[1][3], pose[2][3]]
    return getCartesianJointPosResponse(pose)

def jointCartesianPos_server():
    rospy.init_node('jointCartesianPos_server')
    global right_kin
    robot = URDF.from_parameter_server(key='robot_description')
    base_link = 'base'
    right_end_link = 'right_gripper'
    right_kin = KDLKinematics(robot, base_link, right_end_link)
    global left_kin
    left_end_link = 'left_gripper'
    left_kin = KDLKinematics(robot, base_link, right_end_link)
    s = rospy.Service('getCartesianJointPos', getCartesianJointPos, handle_jointCartesianPos)
    rospy.spin()

if __name__ == "__main__":
    jointCartesianPos_server()
