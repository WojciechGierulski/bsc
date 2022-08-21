#!/usr/bin/env python
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
import rospy
import PyKDL


def transform_triangle(xt, yt, zt, tf):
    for i in range(3):
        p1 = PyKDL.Vector(xt[i], yt[i], zt[i])
        p1 = tf * p1
        xt[i] = p1.x()
        yt[i] = p1.y()
        zt[i] = p1.z()
    return xt, yt, zt


def create_triangle(triangle):
    pts = []
    for i in range(0, 3):
        p = Point()
        p.x = triangle[i, 0]
        p.y = triangle[i, 1]
        p.z = triangle[i, 2]
        pts.append(p)
    return pts


def create_points(ray):
    p1 = Point()
    p2 = Point()
    p1.x = 0
    p1.y = 0
    p1.z = 0
    p2.x = ray[0]
    p2.y = ray[1]
    p2.z = ray[2]
    return p1, p2


def publish_triangles(meshes, frame="world"):
    pub = rospy.Publisher('stll', MarkerArray, queue_size=100)
    rate = rospy.Rate(10)
    for _ in range(10):
        marker_id = 0
        markers = []
        for mesh in meshes.values():
            for triangle in mesh.triangles:
                m = Marker()
                m.id = marker_id
                m.points = create_triangle(triangle)
                m.pose.orientation.w = 1
                m.header.frame_id = frame
                m.color.a = 1
                m.color.r = 1
                m.scale.x = 1
                m.scale.y = 1
                m.scale.z = 1
                m.type = 11
                markers.append(m)
                marker_id += 1
        msg = MarkerArray()
        msg.markers = markers
        pub.publish(msg)
        rate.sleep()


def publish_rays(rays, frame):
    pub = rospy.Publisher('rays', MarkerArray, queue_size=100)
    rate = rospy.Rate(10)
    for _ in range(10):
        markers = []
        marker_id = 0
        for i, ray in enumerate(rays):
            if i % 10000 == 0:
                m = Marker()
                m.id = marker_id
                m.header.frame_id = frame
                m.color.a = 1
                m.color.b = 1
                m.scale.x = 0.01
                m.scale.y = 0.01
                m.type = 0
                p1, p2 = create_points(ray)
                m.points = [p1, p2]
                markers.append(m)
                marker_id += 1
        msg = MarkerArray()
        msg.markers = markers
        pub.publish(msg)
        rate.sleep()


def publish_points(points, topic_name, frame="head_kinect_rgb_optical_frame"):
    pub = rospy.Publisher(topic_name, Marker, queue_size=100)
    rate = rospy.Rate(10)
    for _ in range(10):
        m = Marker()
        m.header.frame_id = frame
        m.scale.x = 0.0025
        m.scale.y = 0.0025
        m.color.a = 1
        m.type = 8
        points_pub = []
        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            points_pub.append(p)
        m.points = points_pub
        pub.publish(m)
        rate.sleep()


def publish_cube(bounds, topic_name, frame="world"):
    pub = rospy.Publisher(topic_name, Marker, queue_size=100)
    rate = rospy.Rate(10)
    for _ in range(10):
        m = Marker()
        m.header.frame_id = frame
        m.pose.position.x = sum(bounds[:, 0])/2
        m.pose.position.y = sum(bounds[:, 1])/2
        m.pose.position.z = sum(bounds[:, 2])/2
        m.scale.x = bounds[1,0] - bounds[0,0]
        m.scale.y = bounds[1,1] - bounds[0,1]
        m.scale.z = bounds[1,2] - bounds[0,2]
        m.color.a = 0.3
        m.color.r = 1
        m.type = 1
        pub.publish(m)
        rate.sleep()
