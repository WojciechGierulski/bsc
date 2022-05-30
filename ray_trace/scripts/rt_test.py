#!/usr/bin/env python
import roslib;

roslib.load_manifest('velma_task_cs_ros_interface')
import numpy as np
import rospy
import PyKDL
import sys
from stl import mesh
import json
import rospkg
from tr_publisher import publish_triangles, test
from velma_common import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma
from rcprg_ros_utils import exitError
from Initializer import Initializer
import math
from ray_trace_maths import ray_trace_call
import sys
import trimesh

RESOLUTION = [800, 600]
FOCAL_LENGTH = 693


def get_stl_dict(rt_path):
    with open(rt_path+"/stl_data/stl_paths.json") as json_file:
        stl_paths = json.load(json_file)
    with open(rt_path+"/stl_data/stl_origins.json") as json_file:
        stl_origins = json.load(json_file)
    return stl_paths, stl_origins


def convert_paths_to_meshes(stl_paths):
    bounding_boxes = []
    for stl_name, stl_path in stl_paths.items():
        stl_mesh = trimesh.load(stl_path, force='mesh')
        stl_paths[stl_name] = stl_mesh
    return stl_paths

def transform_triangle(xt, yt, zt, tf):
    for i in range(3):
        p1 = PyKDL.Vector(xt[i], yt[i], zt[i])
        p1 = tf * p1
        xt[i] = p1.x()
        yt[i] = p1.y()
        zt[i] = p1.z()
    return xt, yt, zt


def get_transform_origin(origin):
    p = PyKDL.Vector(origin["xyz"][0], origin["xyz"][1], origin["xyz"][2])
    r = PyKDL.Rotation.RPY(origin["rpy"][0], origin["rpy"][1], origin["rpy"][2])
    return PyKDL.Frame(r, p)

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
    matrix[2 ,3] = frame.p.z()
    matrix[3, 3] = 1
    return matrix




def transform_meshes(velma, meshes, stl_origins):
    for mesh_name, mesh in meshes.items():
            tf = velma.getTf('world', mesh_name)
            tf_m = get_transform_origin(stl_origins[mesh_name])
            tf = frame_to_tf_matrix(tf)
            tf_m = frame_to_tf_matrix(tf_m)
            meshes[mesh_name] = mesh.apply_transform(tf_m)
            meshes[mesh_name] = mesh.apply_transform(tf)
    return meshes


def transform_meshes_to_camera_frame(velma, meshes):
    tf = velma.getTf('head_kinect_rgb_optical_frame', 'world')
    tf = frame_to_tf_matrix(tf)
    for mesh_name, mesh in meshes.items():
        meshes[mesh_name] = mesh.apply_transform(tf)
    return meshes

def move_head(velma):
    print("moving head to position: down")
    q_dest = (0, 1.0)
    velma.moveHead(q_dest, 3.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose(velma.getHeadCurrentConfiguration(), q_dest, 0.1):
        exitError(7)

def merge_meshes(meshes):
    meshes = [mesh for mesh in meshes.values()]
    return trimesh.util.concatenate(meshes)



if __name__ == "__main__":
    rospy.init_node('rt_velma', anonymous=False)
    rospack = rospkg.RosPack()
    ray_trace_path = rt_path = rospack.get_path("ray_trace")
    stl_paths, stl_origins = get_stl_dict(ray_trace_path)
    meshes = convert_paths_to_meshes(stl_paths)

    velma = Initializer.initialize_system()
    rospy.sleep(0.5)
    move_head(velma)

    meshes = transform_meshes(velma, meshes, stl_origins)
    publish_triangles(meshes)
    rospy.sleep(1)
    meshes = transform_meshes_to_camera_frame(velma, meshes)
    publish_triangles(meshes, "head_kinect_rgb_optical_frame")
    full_mesh = merge_meshes(meshes)
    ray_trace_call(full_mesh, RESOLUTION, FOCAL_LENGTH)




