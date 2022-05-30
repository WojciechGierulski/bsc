#!/usr/bin/env python
from geometry_msgs.msg import Point, PointStamped
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


def test(velma):
    pub = rospy.Publisher('test', MarkerArray, queue_size=100)
    rate = rospy.Rate(10)
    for _ in range(100):
        print("a")
        msg = MarkerArray()
        m = Marker()
        m.id = 1
        m.points = create_triangle([1, 0, 0.3], [1, 0, 0.7], [1, 0, 0.9])
        m.pose.orientation.z = 1
        m.header.frame_id = 'world'
        m.color.a = 1
        m.color.r = 1
        m.scale.x = 1
        m.scale.y = 1
        m.scale.z = 1
        m.type = 11
        msg.markers = [m]
        pub.publish(msg)
        rate.sleep()
    tf = velma.getTf('head_kinect_rgb_optical_frame', 'world')
    x,y,z = transform_triangle([0.5, 0.7, 0.8], [0.4, 0.2, 0.3], [0.8, 0.6, 0.2], tf)
    rospy.sleep(2)
    for _ in range(10):
        break
        msg = MarkerArray()
        m = Marker()
        m.id = 1
        m.points = create_triangle(x,y,z)
        m.pose.orientation.z = 1
        m.header.frame_id = 'head_kinect_rgb_optical_frame'
        m.color.a = 255
        m.color.r = 255
        m.scale.x = 1
        m.scale.y = 1
        m.scale.z = 1
        m.type = 11
        msg.markers = [m]
        pub.publish(msg)
        rate.sleep()


def create_triangle(x, y, z):
    pts = []
    for i in range(0, 3):
        p = Point()
        p.x = x[i]
        p.y = y[i]
        p.z = z[i]
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



def publish_triangles(cuboids, frame="world"):
    pub = rospy.Publisher('stll', MarkerArray, queue_size=100)
    rate = rospy.Rate(10)
    for _ in range(10):
        markers = []
        marker_id = 0
        for cuboid in cuboids:
            for i, (xt, yt, zt) in enumerate(zip(cuboid.x, cuboid.y, cuboid.z)):
                m = Marker()
                m.id = marker_id
                m.points = create_triangle(-xt, -yt, zt)
                m.pose.orientation.z = 1
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
                m.color.a = 255
                m.color.b = 255
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
