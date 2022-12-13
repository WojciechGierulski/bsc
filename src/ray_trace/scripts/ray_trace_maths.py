#!/usr/bin/env python
import numpy
import numpy as np
from geometry_msgs.msg import Point
from tr_publisher import publish_rays, publish_points
import PyKDL
import rospy
from ray_trace.srv import *
import trimesh
import time


def ray_trace_call(mesh, resolution, focal_length):
    rays, origins = generate_rays(resolution, focal_length)
    publish_rays(rays, "head_kinect_rgb_optical_frame")
    start = time.time()
    intersection_points, index_ray, index_tri = mesh.ray.intersects_location(origins, rays, multiple_hits=False)
    end = time.time()
    t = str(end-start)
    print( "Time taken: " + t )
    return intersection_points


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
