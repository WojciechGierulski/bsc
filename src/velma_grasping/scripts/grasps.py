import PyKDL
import math
import copy
from tf_maths import *
import sys
import numpy as np

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    dot = np.dot(v1,v2)
    lenSq1 = np.dot(v1,v1)
    lenSq2 = np.dot(v2,v2)
    angle = math.acos(dot / math.sqrt(lenSq1 * lenSq2))
    return angle

def choose_ang(frame, ang):
    f_test1 = copy.deepcopy(frame)
    f_test2 = copy.deepcopy(frame)
    f_test1.M.DoRotX(ang)
    f_test2.M.DoRotX(-1 * ang)
    pt = PyKDL.Vector(0, 0, 1)
    p1 = f_test1 * pt
    p2 = f_test2 * pt
    if p1.z() > p2.z():
        ang = ang
    else:
        ang = ang * -1
    return ang

class GraspGenerator():

    """
    input - transform matrix 4x4
    output - PyKDL frame
    """

    def __init__(self):
        self.grasps ={
            "Rubber": self.rubber_grasp,
            "Pen1": self.pen1_grasp,
            "Pen2": self.pen1_grasp,
            "BoxLong": self.boxlong_grasp,
            "BoxSquare": self.boxsquare_grasp,
            "GlassWine": self.glasswine_grasp,
            "Mug": self.mug_grasp,
            "Cola": self.cola_grasp,
            "VelmaCylinder": self.velma_cylinder_grasp,
            "VelmaBox1": self.velma_box1_grasp,
            "VelmaBox2": self.velma_box2_grasp,
            "VelmaBowl": self.velma_bowl_grasp
        }

    def rubber_grasp(self, pose, hand):
        pass

    def pen1_grasp(self, pose, hand):
        frames = []
        frame = tf_matrix_to_PyKDLFrame(pose)
        p = PyKDL.Vector(0,0,1)
        p = frame * p
        ang = angle_between(np.array([0, 0, 1]), np.array([p.x(), p.y(), p.z()]) - pose[0:3, 3])
        ang = choose_ang(frame, ang)
        frame.M.DoRotX(ang)
        p = frame * PyKDL.Vector(0.0, 0.0, 0.2)
        frame.p = p
        frame1 = copy.deepcopy(frame)
        frame1.M.DoRotY(math.radians(180))
        frames.append(frame1)
        #frame.M.DoRotZ(math.radians(180))
        #frame.M.DoRotY(math.radians(180))
        #frames.append(frame)
        return frames, ["gripper", (1.3*math.pi/3, 1.3*math.pi/3, 1.3*math.pi/3, 0), "forward", 0.15, "gripper", (math.pi/2, math.pi/2, math.pi/2, 0), "forward", -0.2]

    def cola_grasp(self, pose, hand):
        frames = []
        frame = tf_matrix_to_PyKDLFrame(pose)
        p = frame * PyKDL.Vector(0.0, 0, 0.08)
        frame.p = p
        for ang in range(0, 360, 15):
            new_frame = copy.deepcopy(frame)
            new_frame.M.DoRotZ(math.radians(ang))
            p = new_frame * PyKDL.Vector(-0.2, 0, 0.0)
            new_frame.p = p
            if hand == "right":
                new_frame.M.DoRotY(math.radians(90))
            elif hand == "left":
                new_frame.M.DoRotX(math.radians(180))
                new_frame.M.DoRotY(math.radians(90))

            frames.append(new_frame)
        return frames, ["gripper", (0, 0, 0, 0), "forward", 0.20, "gripper", (math.pi*4/5, math.pi*4/5, math.pi*4/5, 0), "up", 0.3]

    def mug_grasp(self, pose, hand):
        """
        input - transform matrix 4x4
        output - PyKDL frame list, gripper sequence
        """
        frames = []
        frame = tf_matrix_to_PyKDLFrame(pose)
        frame.M.DoRotZ(math.radians(180))
        frame.M.DoRotY(math.radians(90))
        p = frame * PyKDL.Vector(0.0, 0, -0.3)
        frame.p = p
        if hand == "right":
            frames.append(copy.deepcopy(frame))
        elif hand == "left":
            frame.M.DoRotX(math.radians(180))
            frame.M.DoRotY(math.radians(180))
            frames.append(copy.deepcopy(frame))
        return frames, ["gripper", (0, 0, 0, 0), "forward", 0.2, "gripper", (math.pi, math.pi, math.pi, 0), "up", 0.3]

    def boxlong_grasp(self, pose, hand):
        # TODO
        frames = []
        frame = tf_matrix_to_PyKDLFrame(pose)
        for i in range(4):
            curr_frame = copy.deepcopy(frame)
            curr_frame.M.DoRotX(math.radians(i * 90))
            p = curr_frame * PyKDL.Vector(0.0, 0, 0.3)
            curr_frame.p = p
            curr_frame.M.DoRotZ(math.radians(180))
            if hand == "right":
                pass
            frames.append(curr_frame)
        return frames, ["gripper", (0, 0, 0, 0), "forward", 0.2, "gripper", (math.pi/2, math.pi/2, math.pi/2, 0), "forward", -0.3]

    def boxsquare_grasp(self, transform, hand):
        x = transform[0, 3]
        y = transform[1, 3]
        z = transform[2, 3]
        z += 0.2

        base_rot = PyKDL.Rotation(1,0,0,0,1,0,0,0,1)
        base_rot.DoRotY(math.radians(180))

        rots = []
        for angle in range(0, 360, 60):
            new_rot = copy.deepcopy(base_rot)
            new_rot.DoRotZ(math.radians(angle))
            rots.append(new_rot)

        frames = []
        vec = PyKDL.Vector(x, y, z)
        for rot in rots:
            frame = PyKDL.Frame(rot, vec)
            frames.append(frame)
        return frames, ["gripper", (0, 0, 0, 0), "forward", 0.2, "gripper", (math.pi/2, math.pi/2, math.pi/2, 0), "forward", -0.2]


    def glasswine_grasp(self, pose, hand):
        frames = []
        frame = tf_matrix_to_PyKDLFrame(pose)
        p = frame * PyKDL.Vector(0.0, 0, 0.08)
        frame.p = p
        for ang in range(0, 360, 15):
            new_frame = copy.deepcopy(frame)
            new_frame.M.DoRotZ(math.radians(ang))
            p = new_frame * PyKDL.Vector(-0.2, 0, 0.0)
            new_frame.p = p
            if hand == "right":
                new_frame.M.DoRotY(math.radians(90))
            elif hand == "left":
                new_frame.M.DoRotX(math.radians(180))
                new_frame.M.DoRotY(math.radians(90))
            frames.append(new_frame)
        return frames, ["gripper", (0, 0, 0, 0), "forward", 0.20, "gripper", (math.pi*3.5/5, math.pi*3.5/5, math.pi*3.5/5, 0), "up", 0.3]

    def velma_bowl_grasp(self, pose, hand):
        frames = []
        frame = tf_matrix_to_PyKDLFrame(pose)
        for ang in np.linspace(0,360,12):
            f = copy.deepcopy(frame)
            f.M.DoRotZ(math.radians(ang))
            p = f * PyKDL.Vector(-0.08, 0.0, 0.0)
            f.p = p
            f.M.DoRotY(-1*math.radians(15))
            p = f * PyKDL.Vector(0.0, 0.0, 0.28)
            f.p = p
            f.M.DoRotY(math.radians(180))
            f1 = copy.deepcopy(f)
            f1.M.DoRotZ(math.radians(90))
            f.M.DoRotZ(-1*math.radians(90))
            frames.append(f1)
            frames.append(f)

        return frames, ["gripper", (0, 0, 0, 0), "forward", 0.20, "gripper", (math.pi*3/5, math.pi*3/5, math.pi*3/5, 0), "forward", -0.25]

    def velma_box1_grasp(self, pose, hand):
        frames = []
        frame = tf_matrix_to_PyKDLFrame(pose)
        p = PyKDL.Vector(0, 0, 1)
        p = frame * p
        ang = angle_between(np.array([0, 0, 1]), np.array([p.x(), p.y(), p.z()]) - pose[0:3, 3])
        ang = choose_ang(frame, ang)
        frame.M.DoRotX(ang)
        p = frame * PyKDL.Vector(0.0, 0, 0.02)
        frame.p = p
        frame.M.DoRotY(math.radians(180))
        p = frame * PyKDL.Vector(0.0, 0.0, -0.2)
        frame.p = p
        for a in [0,90,180,270]:
            f = copy.deepcopy(frame)
            f.M.DoRotZ(math.radians(a))
            frames.append(f)

        return frames, ["gripper", (0, 0, 0, 0), "forward", 0.20, "gripper", (math.pi*3/5, math.pi*3/5, math.pi*3/5, 0), "forward", -0.25]


    def velma_box2_grasp(self, pose, hand):
        frames = []
        frame = tf_matrix_to_PyKDLFrame(pose)
        p = PyKDL.Vector(0, 0, 1)
        p = frame * p
        ang = angle_between(np.array([0, 0, 1]), np.array([p.x(), p.y(), p.z()]) - pose[0:3, 3])
        ang = choose_ang(frame, ang)
        frame.M.DoRotX(ang)
        p = frame * PyKDL.Vector(0.0, 0.0, 0.2)
        frame.p = p
        frame.M.DoRotY(math.radians(180))
        f1 = copy.deepcopy(frame)
        f1.M.DoRotZ(math.radians(180))
        frames.append(frame)
        frames.append(f1)
        return frames, ["gripper", (0, 0, 0, 0), "forward", 0.20, "gripper", (math.pi*3/5, math.pi*3/5, math.pi*3/5, 0), "forward", -0.25]

    def velma_cylinder_grasp(self, pose, hand):
        frames = []
        frame = tf_matrix_to_PyKDLFrame(pose)
        p = PyKDL.Vector(0, 0, 1)
        p = frame * p
        ang = angle_between(np.array([0, 0, 1]), np.array([p.x(), p.y(), p.z()]) - pose[0:3, 3])
        ang = choose_ang(frame, ang)
        frame.M.DoRotX(ang)
        for ang in range(0, 360, 15):
            new_frame = copy.deepcopy(frame)
            new_frame.M.DoRotZ(math.radians(ang))
            p = new_frame * PyKDL.Vector(-0.2, 0, 0.0)
            new_frame.p = p
            if hand == "right":
                new_frame.M.DoRotY(math.radians(90))
            elif hand == "left":
                new_frame.M.DoRotX(math.radians(180))
                new_frame.M.DoRotY(math.radians(90))

            frames.append(new_frame)
        return frames, ["gripper", (0, 0, 0, 0), "forward", 0.20, "gripper", (math.pi*4/5, math.pi*4/5, math.pi*4/5, 0), "up", 0.3]

