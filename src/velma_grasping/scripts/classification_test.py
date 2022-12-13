#!/usr/bin/env python3


from visual_analysis_functions import process_kinect_data, classify_pc
from objects_databse import DataBase
import pickle
import open3d as o3d
import rospkg

rospack = rospkg.RosPack()

#filepath1 = "../../../data/2_tf2.pickle"
#filepath2 = "../../../data/2.pickle"

filepath1 = f"{rospack.get_path('velma_grasping')}/data/2_tf2.pickle"
filepath2 = f"{rospack.get_path('velma_grasping')}/data/2.pickle"

with open(filepath1, "rb") as file:
    tf = pickle.load(file, encoding='latin1')
with open(filepath2, "rb") as file:
    data = pickle.load(file, encoding='latin1')

db = DataBase()
db.load_db()

clusters = process_kinect_data(data, tf)
print(clusters)


for cluster in clusters:
    transform, fitness, name = classify_pc(cluster, db)
    print("RESULT")
    o3d.visualization.draw_geometries([cluster, db.objects[name].pc.transform(transform)])