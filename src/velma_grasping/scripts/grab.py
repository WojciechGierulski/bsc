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
from tf_maths import tf_msg_to_matrix, frame_to_tf_matrix
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
from Publishers import publish_detected_object_tf, publish_pose_arr
from grasps import GraspGenerator
from Functions import check_which_hand
from sequence_execution import SequenceExecutor


def ListToTypeList(list, type):
    new_list = []
    for el in list:
        el2 = type()
        el2.data = el
        new_list.append(el2)
    return new_list

def TypeListToList(list):
    return [el.data for el in list]


def LongTransformationListToNumpy(list):
    return [np.array(el.data).reshape((4, 4)) for el in list]

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


def send_classification_request(world_to_cam_transform, pc, calib_tf=None):
    if calib_tf is None:
        calib_tf = np.array([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]).reshape((4,4))
    rospy.wait_for_service('classify')
    try:
        srv = rospy.ServiceProxy('classify', classify)
        world_to_cam_transform_ros = ListToTypeList(world_to_cam_transform.flatten().tolist(), Float32)
        calib_tf_ros = ListToTypeList(calib_tf.flatten().tolist(), Float32)
        resp = srv(world_to_cam_transform_ros, calib_tf_ros, pc)
        return TypeListToList(resp.classes), TypeListToList(resp.scores), LongTransformationListToNumpy(resp.transformations)
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
    print("Closing grippers")
    GripperMoves.close_grippers(velma, 'both')
    rospy.sleep(1)
    print("Moving to init pos")
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
    JointImpMoves.move_head((0, 0), velma)
    JointImpMoves.move_to_init_pos(velma)

    JointImpMoves.move_head((0, 0.85), velma)

    # Get pc
    print("Requesting classification")
    point_cloud = rospy.wait_for_message(PARAMS["pc_topic"], PointCloud2, timeout=None)
    world_to_cam_transform = frame_to_tf_matrix(get_tf(PARAMS["world_frame"], PARAMS["camera_frame"]))
    classes, scores, transformations = send_classification_request(world_to_cam_transform, point_cloud, calib_tf)
    transformations = [np.linalg.inv(transform) for transform in transformations]
    print(classes)
    rospy.Timer(rospy.Duration(1), lambda x: publish_detected_object_tf(transformations, classes))

    if OBJECT_NAME == "Pen1" or OBJECT_NAME == "Pen2":
        try:
            OBJECT_NAME = "Pen1"
            idx = classes.index(OBJECT_NAME)
        except:
            try:
                OBJECT_NAME = "Pen2"
                idx = classes.index(OBJECT_NAME)
            except:
                print("No such object")
                sys.exit()
    else:
        try:
            idx = classes.index(OBJECT_NAME)
        except:
            print("No such object")
            sys.exit()

    hand = check_which_hand(velma, transformations[idx])
    hand = "right"

    GG = GraspGenerator()
    frames, sequence = GG.grasps[OBJECT_NAME](transformations[idx], hand)
    publish_pose_arr(frames, "grasps")


    frames = JointImpMoves.gripper_to_joint_7_transform(velma, frames, hand)
    publish_pose_arr(frames, "grasps")
    iks, torsos = JointImpMoves.get_IK_for_frames(hand, frames, solver)
    qs = JointImpMoves.transform_iks_and_torsos_to_q(iks, torsos, hand)
    JointImpMoves.move_with_planning(velma, qs, planner, hand)
    SequenceExecutor.execute_sequence(sequence, velma, hand, solver)
    print("Done")

    rospy.spin()