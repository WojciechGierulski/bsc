import time

import numpy as np

from visual_analysis import process_kinect_data, classify
from objects_databse import DataBase
import pickle
import open3d as o3d
import matplotlib.pyplot as plt
import cv2 as cv


with open("../../../data/2_tf2.pickle", "rb") as file:
    tf = pickle.load(file, encoding='latin1')
with open("../../../data/2.pickle", "rb") as file:
    data = pickle.load(file, encoding='latin1')

db = DataBase()
db.load_db()

clusters = process_kinect_data(data, tf)
print(clusters)
for i, cluster in enumerate(clusters):
    print(i)
    o3d.io.write_point_cloud(f"data/pc{i}_real.ply", cluster)


for cluster in clusters:
    transform, fitness, name = classify(cluster, db)
    o3d.visualization.draw_geometries([cluster])
    o3d.visualization.draw_geometries([cluster.transform(transform), db.objects[name].pc, o3d.geometry.TriangleMesh.create_coordinate_frame(0.1)])