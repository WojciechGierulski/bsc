import PyKDL
import math
import copy
from tf_maths import *

class GraspGenerator():

    """
    input - transform matrix 4x4
    output - PyKDL frame
    """

    def __init__(self):
        self.grasps ={
            "Rubber": self.rubber_grasp,
            "Pen1": self.pen1_grasp,
            "Pen2": self.pen2_grasp,
            "BoxLong": self.boxlong_grasp,
            "BoxSquare": self.boxsquare_grasp,
            "GlassWine": self.glasswine_grasp,
            "Mug": self.mug_grasp,
            "Cola": self.cola_grasp
        }

    def rubber_grasp(self, pose, hand):
        pass

    def pen1_grasp(self, pose, hand):
        frames = []
        frame = tf_matrix_to_PyKDLFrame(pose)


    def pen2_grasp(self, pose, hand):
        pass

    def cola_grasp(self, pose, hand):
        frames = []
        frame = tf_matrix_to_PyKDLFrame(pose)
        p = frame * PyKDL.Vector(0.0, 0, 0.08)
        frame.p = p
        for ang in range(0, 360, 30):
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
        return frames, ["gripper", (0, 0, 0, 0), "forward", 0.25, "gripper", (math.pi*3/5, math.pi*3/5, math.pi*3/5, 0), "up", 0.3]

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
        return frames, ["gripper", (0, 0, 0, 0), "forward", 0.2, "gripper", (math.pi*3/5, math.pi*3/5, math.pi*3/5, 0), "up", 0.3]

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
        p = frame * PyKDL.Vector(0.0, 0, 0.12)
        frame.p = p
        for ang in range(0, 360, 30):
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
        return frames, ["gripper", (0, 0, 0, 0), "forward", 0.25, "gripper", (math.pi*3/5, math.pi*3/5, math.pi*3/5, 0), "up", 0.3]

