#!/usr/bin/env python
import sys
import numpy as np
import rospy
import PyKDL
import json
import rospkg
from tr_publisher import publish_triangles, publish_cube, publish_points
from ray_trace_maths import ray_trace_call
import trimesh
from sensor_msgs.msg import PointCloud2
import ros_numpy
from tf_getter import TfGetter
from ray_trace.srv import ray_trace, ray_traceResponse
from geometry_msgs.msg import Transform
from std_msgs.msg import Int16
from velma_grasping.srv import classify


def load_params(rt_path):
    with open(rt_path+"/config/params.json") as file:
        params = json.load(file)
        RESOLUTION = params["resolution"]
        FOCAL_LENGTH = params["focal_length"]
        PC_TOPIC = params["pc_topic"]
        WORLD_FRAME = params["world_frame"]
        CAMERA_FRAME = params["camera_frame"]
        return RESOLUTION, FOCAL_LENGTH, PC_TOPIC, WORLD_FRAME, CAMERA_FRAME

def get_points_in_bounds(pc, bounds):
    min_x = bounds[0, 0]
    max_x = bounds[1, 0]
    min_y = bounds[0, 1]
    max_y = bounds[1, 1]
    min_z = bounds[0, 2]
    max_z = bounds[1, 2]
    mask = (pc[:, 0] > min_x) & (pc[:, 0] < max_x) & (pc[:, 1] > min_y) & (pc[:, 1] < max_y) & (pc[:, 2] > min_z) & (
                pc[:, 2] < max_z)
    return pc[mask]


def get_inflated_bounds(pc):
    bounds = pc.bounds
    bounds[0, :] -= 0.05
    bounds[1, :] += 0.05
    return bounds


def get_stl_dict(rt_path):
    with open(rt_path + "/stl_data/stl_paths.json") as json_file:
        stl_paths = json.load(json_file)
    with open(rt_path + "/stl_data/stl_origins.json") as json_file:
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
    matrix[2, 3] = frame.p.z()
    matrix[3, 3] = 1
    return matrix


def transform_pc(tf, pc):
    tf = frame_to_tf_matrix(tf)
    pc.apply_transform(tf)


def transform_meshes(meshes, stl_origins, tf_getter):
    for mesh_name, mesh in meshes.items():
        tf = tf_getter.get_tf(WORLD_FRAME, mesh_name[0:-2]) # crop last "_{nr}, ex.: gripper_link_2"
        tf_m = get_transform_origin(stl_origins[mesh_name])
        tf = frame_to_tf_matrix(tf)
        tf_m = frame_to_tf_matrix(tf_m)
        meshes[mesh_name] = mesh.apply_transform(tf_m)
        meshes[mesh_name] = mesh.apply_transform(tf)
    return meshes


def transform_meshes_to_camera_frame(meshes, tf_getter):
    tf = tf_getter.get_tf(CAMERA_FRAME, WORLD_FRAME)
    tf = frame_to_tf_matrix(tf)
    for mesh_name, mesh in meshes.items():
        meshes[mesh_name] = mesh.apply_transform(tf)
    return meshes

def merge_meshes(meshes):
    meshes = [mesh for mesh in meshes.values()]
    return trimesh.util.concatenate(meshes)


def callback_t(data):
    print(ros_numpy.numpify(data))

def tf_matrix_to_tf_msg(matrix):
    tf = Transform()
    tf.translation.x = matrix[0, 3]
    tf.translation.y = matrix[1, 3]
    tf.translation.z = matrix[2, 3]
    v1 = np_to_pykdl_vector(matrix[0, 0:3])
    v2 = np_to_pykdl_vector(matrix[1, 0:3])
    v3 = np_to_pykdl_vector(matrix[2, 0:3])

    x, y, z, w = PyKDL.Rotation(v1, v2, v3).GetQuaternion()
    tf.rotation.x = x
    tf.rotation.y = y
    tf.rotation.z = z
    tf.rotation.w = w
    return tf

def np_to_pykdl_vector(l):
    v = PyKDL.Vector(l[0], l[1], l[2])
    return v


def handle_request(req):
    global WORLD_FRAME, CAMERA_FRAME
    print("Got request")
    publish = req.publish_results.data


    stl_paths, stl_origins = get_stl_dict(ray_trace_path)
    meshes = convert_paths_to_meshes(stl_paths)

    tf_getter = TfGetter()

    meshes = transform_meshes(meshes, stl_origins, tf_getter)
    if publish:
        publish_triangles(meshes)
    rospy.sleep(0.1)
    meshes = transform_meshes_to_camera_frame(meshes, tf_getter)
    if publish:
        publish_triangles(meshes, CAMERA_FRAME)
    full_mesh = merge_meshes(meshes)
    intersections = ray_trace_call(full_mesh, RESOLUTION, FOCAL_LENGTH)
    if len(intersections) == 0:
        print "No intersection points"
        i = Int16()
        i.data = 1
        return ray_traceResponse(i, Transform())
    model_pc = trimesh.points.PointCloud(intersections)
    transform_pc(tf_getter.get_tf(WORLD_FRAME, CAMERA_FRAME), model_pc)
    if publish:
        publish_points(model_pc.vertices, "intersections", WORLD_FRAME)
    inflated_bounds = get_inflated_bounds(model_pc)
    if publish:
        publish_cube(inflated_bounds, "bounds", WORLD_FRAME)

    points_camera = rospy.wait_for_message(PC_TOPIC, PointCloud2)
    points_camera = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(points_camera)
    pc_camera = trimesh.points.PointCloud(points_camera)
    transform_pc(tf_getter.get_tf(WORLD_FRAME, CAMERA_FRAME), pc_camera)
    cropped_points = get_points_in_bounds(pc_camera.vertices, inflated_bounds)
    if publish:
        publish_points(cropped_points, "cropped_camera_pc", WORLD_FRAME)
    pc_camera = trimesh.points.PointCloud(cropped_points)
    matrix, transformed, cost = trimesh.registration.icp(model_pc.vertices, pc_camera.vertices, max_iterations=100, threshold=0.000001)
    print "Finished processing"
    i = Int16()
    i.data = 0
    tf_msg = tf_matrix_to_tf_msg(matrix)
    return ray_traceResponse(i, tf_msg)

if __name__ == "__main__":
    rospy.init_node('ray_trace_server')
    rospack = rospkg.RosPack()
    ray_trace_path = rospack.get_path("ray_trace")
    # Set params
    RESOLUTION, FOCAL_LENGTH, PC_TOPIC, WORLD_FRAME, CAMERA_FRAME = load_params(ray_trace_path)
    s = rospy.Service('ray_trace', ray_trace, handle_request)
    print("Server ready.")
    rospy.spin()
