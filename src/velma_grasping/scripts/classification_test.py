import time

from visual_analysis import process_kinect_data, classify
from objects_databse import DataBase
import pickle
import open3d as o3d
import matplotlib.pyplot as plt


with open("../../../data/4_tf2.pickle", "rb") as file:
    tf = pickle.load(file, encoding='latin1')
with open("../../../data/4.pickle", "rb") as file:
    data = pickle.load(file, encoding='latin1')

db = DataBase()
db.load_db()

clusters = process_kinect_data(data, tf)

start = time.time()

for cluster in clusters:
    transform, fitness, name = classify(cluster, db)
    o3d.visualization.draw_geometries([cluster])
    o3d.visualization.draw_geometries([cluster.transform(transform), db.objects[name].pc, o3d.geometry.TriangleMesh.create_coordinate_frame(0.1)])

print(time.time()-start)