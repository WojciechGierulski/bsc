#!/usr/bin/env python3

import open3d as o3d
import sys
import os
import rospkg

rospack = rospkg.RosPack()

class Object:
    def __init__(self, name):
        self.name = name
        path = f"{rospack.get_path('velma_grasping')}/models/{name}/{name}.stl"
        mesh = o3d.io.read_triangle_mesh(path)
        pc = self.preprocess(mesh)
        self.pc = pc

    def preprocess(self, mesh):
        # Must return o3d.geometry.PointCloud
        pc = mesh.sample_points_poisson_disk(number_of_points=25000, init_factor=2).paint_uniform_color([1, 0.2, 0])
        return pc


class DataBase():
    def __init__(self):
        self.objects = {}
        self.module = sys.modules[__name__]

    def load_db(self):
        with open(f"{rospack.get_path('velma_grasping')}/database/objects.txt", "r") as file:
            objects_names = file.read().splitlines()
        for name in objects_names:
            if hasattr(self.module, name):
                self.objects[name] = (getattr(self.module, name)(name))
            else:
                self.objects[name] = (Object(name))
