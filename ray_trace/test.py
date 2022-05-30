import trimesh
import pyembree
import numpy as np

vertices = [[0, 0, 0],
            [20, 0, 0],
            [0, 20, 0],
            [20, 20, 0]
            ]
vertices = np.array(vertices)


mesh = trimesh.Trimesh(vertices=vertices, faces=[[0,1,2], [3,2,1]])
#mesh = trimesh.load('cube.stl', force='mesh')

e = np.arange(0,33,1)
print(e)
e=e.reshape((11,3))
print(e)
