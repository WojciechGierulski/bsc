from geometry_msgs.msg import Pose
import PyKDL
import math

def check_which_hand(velma, object_position):
    """
    :type object_position - tf_matrix (4x4 ndarray)
    """
    x = object_position[0, 3]
    y = object_position[1, 3]
    z = object_position[2, 3]

    right = velma.getTf("world", "right_HandGripLink")
    left = velma.getTf("world" ,"left_HandGripLink")

    r_dist = math.sqrt((x-right.p.x())**2 + (y-right.p.y())**2 + (z-right.p.z())**2)
    l_dist = math.sqrt((x-left.p.x())**2 + (y-left.p.y())**2 + (z-left.p.z())**2)

    if r_dist > l_dist:
        return "left"
    else:
        return "right"
