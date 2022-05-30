#!/usr/bin/env python
import numpy
import numpy as np
from geometry_msgs.msg import Point
from ray_trace.msg import Triangle
from tr_publisher import publish_rays
import PyKDL
import rospy
from ray_trace.srv import *
import trimesh
import pyembree


def ray_trace_call(meshes, resolution, focal_length):
    rays, origins = generate_rays(resolution, focal_length)
    publish_rays(rays, "head_kinect_rgb_optical_frame")
    full_mesh, faces = convert_data(meshes)
    full_mesh = trimesh.Trimesh(vertices=full_mesh, faces=faces, use_embree=True)
    intersection_points, index_ray, index_tri = full_mesh.ray.intersects_location(origins, rays, multiple_hits=False)
    print(intersection_points)
    print("END")


def generate_rays(resolution, focal_length):
    ray_total_nr = (resolution[0] + 1) * (resolution[1] + 1)
    rays = np.empty([ray_total_nr, 3], dtype=np.float32)
    i = 0
    for x_cord in range(-resolution[0] // 2, resolution[0] // 2 + 1):
        for y_cord in range(-resolution[1] // 2, resolution[1] // 2 + 1):
            ray = np.array([x_cord, y_cord, focal_length])
            rays[i, :] = ray
            i += 1
    origins = np.zeros((len(rays), 3))
    return rays, origins


def convert_data(meshes):
    full_mesh = np.zeros((0, 3))
    for mesh in meshes:
        x = numpy.reshape(mesh.x, (3 * len(mesh.x), 1))
        y = numpy.reshape(mesh.y, (3 * len(mesh.y), 1))
        z = numpy.reshape(mesh.x, (3 * len(mesh.z), 1))
        part_mesh = np.concatenate((x, y, z), axis=1)
        full_mesh = np.concatenate((full_mesh, part_mesh), axis=0)
    faces = np.arange(0, len(full_mesh), 1)
    print(full_mesh[0:10, :])
    faces = faces.reshape((len(full_mesh) // 3, 3))
    return full_mesh, faces
