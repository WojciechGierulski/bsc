#!/usr/bin/env python
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
import rospy



def create_triangle(x, y, z):
    pts = []
    for i in range(0, 3):
        p = Point()
        p.x = x[i]
        p.y = y[i]
        p.z = z[i]
        pts.append(p)
    return pts


def publish_triangles(cuboids):
    pub = rospy.Publisher('stll', MarkerArray, queue_size=100)
    rate = rospy.Rate(10)
    for _ in range(10):
        markers = []
        marker_id = 0
        for cuboid in cuboids:
            for i, (xt, yt, zt) in enumerate(zip(cuboid.x, cuboid.y, cuboid.z)):
                m = Marker()
                m.id = marker_id
                m.points = create_triangle(xt, yt, zt)
                m.pose.orientation.z = 1
                m.header.frame_id = "world"
                m.color.a = 255
                m.color.r = 255
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