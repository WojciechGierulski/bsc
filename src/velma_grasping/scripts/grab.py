#!/usr/bin/env python
import sys
import time

import roslib;

roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import PyKDL
import rospkg
from velma_common import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma
from rcprg_ros_utils import exitError
from Initializer import Initializer
from sensor_msgs.msg import PointCloud2
from ray_trace.srv import ray_trace
from tf_maths import tf_msg_to_matrix
from std_msgs.msg import Bool
from Initializer import Initializer
from GripperMoves import GripperMoves
from JointImpMoves import JointImpMoves
from EnvScanner import EnvScanner
import json
import tf
import numpy as np
from velma_grasping.srv import classify
from std_msgs.msg import Float32

def list2Float32List(list):
    new_list = []
    for i in list:
        a = Float32()
        a.data = i
        new_list.append(a)
    return new_list


def load_params(rospack):
    path = rospack.get_path("velma_grasping")
    f = open(path+"/config/config.json")
    data = json.load(f)
    return data

def get_tf_matrix(publish=False):
    rospy.wait_for_service('ray_trace')
    try:
        srv = rospy.ServiceProxy('ray_trace', ray_trace)
        b = Bool()
        b.data = publish
        resp = srv(b)
        if resp.status.data == 1:
            print ("No intersection points!")
            return None
        elif resp.status.data == 0:
            matrix = tf_msg_to_matrix(resp.transform)
            return matrix
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit()

def frame_to_tf_matrix(frame):
    matrix = np.zeros((4, 4))
    matrix[0, 0] = frame.M[0, 0]
    matrix[1, 0] = frame.M[1, 0]
    matrix[0, 1] = frame.M[0, 1]
    matrix[1, 1] = frame.M[1, 1]
    matrix[2, 0] = frame.M[2, 0]
    matrix[2, 1] = frame.M[2, 1]
    matrix[2, 2] = frame.M[2, 2]
    matrix[0, 2] = frame.M[0, 2]
    matrix[1, 2] = frame.M[1, 2]
    matrix[0, 3] = frame.p.x()
    matrix[1, 3] = frame.p.y()
    matrix[2, 3] = frame.p.z()
    matrix[3, 3] = 1
    return matrix


def send_classification_request(world_to_cam_transform, calib_tf, pc):
    rospy.wait_for_service('classify')
    try:
        srv = rospy.ServiceProxy('classify', classify)
        world_to_cam_transform_ros = list2Float32List(world_to_cam_transform.flatten().tolist())
        calib_tf_ros = list2Float32List(calib_tf.flatten().tolist())
        resp = srv(world_to_cam_transform_ros, calib_tf_ros, pc)
        print(resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit()

def get_tf(src, dst):
    global listener
    listener.waitForTransform(src, dst, rospy.Time(), rospy.Duration(1))
    pos, rot = listener.lookupTransform(src, dst, rospy.Time(0))
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3]), PyKDL.Vector(pos[0], pos[1], pos[2]))

if __name__ == "__main__":
    OCTOMAP_TYPE = sys.argv[1].lower()
    OBJECT_NAME = sys.argv[2]

    rospy.init_node('rt_velma', anonymous=False)
    rospack = rospkg.RosPack()
    listener = tf.TransformListener()
    rt_path = rospack.get_path("ray_trace")
    PARAMS = load_params(rospack)

    print("Initializing system")
    velma = Initializer.initialize_system()
    planner = Initializer.get_planner(velma)
    solver = Initializer.get_solver()
    GripperMoves.close_grippers(velma, 'both')
    rospy.sleep(1)
    JointImpMoves.move_to_init_pos(velma)
    rospy.sleep(1)
    if OCTOMAP_TYPE == "online":
        print("Scanning Env")
        EnvScanner.scan_env(velma)

    print("Processing octomap")
    octomap = Initializer.get_octomap()
    Initializer.process_octomap(planner, octomap)

    print("Moving to calib pose")
    JointImpMoves.move_to_calib_pose(velma, planner, rt_path, 1)
    GripperMoves.open_grippers(velma, 'right')

    print("Getting calibration transform")
    calib_tf = get_tf_matrix(True)
    print(calib_tf)

    # Move back to init pose
    print("Moving back to init pos")
    GripperMoves.close_grippers(velma, 'right')
    JointImpMoves.move_to_init_pos(velma)

    # Get pc
    point_cloud = rospy.wait_for_message(PARAMS["pc_topic"], PointCloud2, timeout=None)
    world_to_cam_transform = frame_to_tf_matrix(get_tf(PARAMS["world_frame"], PARAMS["camera_frame"]))
    send_classification_request(world_to_cam_transform, calib_tf, point_cloud)





