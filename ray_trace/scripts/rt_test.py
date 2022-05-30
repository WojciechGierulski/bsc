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

RESOLUTION = [400, 300]
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
        a = trimesh.load(stl_path, force='mesh')
        a.show()
        stl_mesh = mesh.Mesh.from_file(stl_path)
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


def transform_meshes(velma, meshes, stl_origins):
    j=0
    for mesh_name, mesh in meshes.items():
            tf = velma.getTf('world', mesh_name)
            t = tf.p
            r, p, y = tf.M.GetRPY()
            for i, (xt, yt, zt) in enumerate(zip(mesh.x, mesh.y, mesh.z)):
                t_m = get_transform_origin(stl_origins[mesh_name])
                xt, yt, zt = transform_triangle(xt, yt, zt, t_m)
                xt, yt, zt = transform_triangle(xt, yt, zt, tf)
                meshes[mesh_name].x[i] = xt
                meshes[mesh_name].y[i] = yt
                meshes[mesh_name].z[i] = zt
                j += 1
            #mesh.rotate([0,0,1], math.radians(180))
    print(j)
    return meshes

def transform_meshes_to_camera_frame(velma, meshes):
    tf = velma.getTf('head_kinect_rgb_optical_frame', 'world')
    for mesh_name, mesh in meshes.items():
            for i, (xt, yt, zt) in enumerate(zip(mesh.x, mesh.y, mesh.z)):
                xt, yt, zt = transform_triangle(xt, yt, zt, tf)
                meshes[mesh_name].x[i] = xt
                meshes[mesh_name].y[i] = yt
                meshes[mesh_name].z[i] = zt
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
    #publish_triangles(list(meshes.values()), "world")
    rospy.sleep(1)
    meshes = transform_meshes_to_camera_frame(velma, meshes)
    publish_triangles(list(meshes.values()), "head_kinect_rgb_optical_frame")

    ray_trace_call(np.array(meshes.values()), RESOLUTION, FOCAL_LENGTH)




