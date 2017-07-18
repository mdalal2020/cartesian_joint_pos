import sys
import rospy
from cartesian_joint_pos.srv import *
import numpy as np

def jointCartesianPos_client(name, tip):
    rospy.wait_for_service('getCartesianJointPos')
    try:
        jointCartesianPos = rospy.ServiceProxy('getCartesianJointPos', getCartesianJointPos, persistent=True)
        resp = jointCartesianPos(name, tip)
        return resp.pose
    except rospy.ServiceException as e:
        print e

if __name__ == "__main__":
    print(jointCartesianPos_client('right', '_gripper'))
