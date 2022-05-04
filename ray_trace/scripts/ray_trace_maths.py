import numpy as np
from tr_publisher import publish_rays
import PyKDL


# Triangles intersections

def get_bounding_box(mesh):
    maxx = mesh.x.max()
    minx = mesh.x.min()
    maxy = mesh.y.max()
    miny = mesh.y.min()
    maxz = mesh.z.max()
    minz = mesh.z.min()
    return [minx, maxx, miny, maxy, minz, maxz]


def ray_triangle_intersect(ray_origin, ray_vector, A, B, C):
    E1 = B - A
    E2 = C - A
    N = np.cross(E1, E2)
    det = -np.dot(ray_vector, N)
    invdet = 1 / det
    A0 = ray_origin - A
    DA0 = np.cross(A0, ray_vector)
    u = np.dot(E2, DA0) * invdet
    v = -np.dot(E1, DA0) * invdet
    t = np.dot(A0, N) * invdet
    result = det >= 1e-6 and t >= 0.0 and u >= 0.0 and v >= 0.0 and (u + v) <= 1.0
    if result:
        return ray_origin + t * ray_vector
    else:
        return None


def consider_z_buffer(intersections, z_min, z_max):
    t1 = intersections[:, 2] > z_min
    t2 = intersections[:, 2] < z_max
    t = np.logical_and(t1, t2)
    return intersections[t]


def ray_trace(meshes, resolution, focal_length):
    intersections = []
    rays = generate_rays(resolution, focal_length)
    publish_rays(rays, "head_kinect_rgb_optical_frame")
    i = 0
    for ray in rays:
        for mesh in meshes:
            b_box = get_bounding_box(mesh)
            if ray_bounding_box_intersection(PyKDL.Vector(b_box[0], b_box[2], b_box[4]),
                                             PyKDL.Vector(b_box[1], b_box[3], b_box[5]), PyKDL.Vector(0, 0, 0),
                                             PyKDL.Vector(ray[0], ray[1], ray[2])):
                for xt, yt, zt in zip(mesh.x, mesh.y, mesh.z):
                    A = np.array([xt[0], yt[0], zt[0]])
                    B = np.array([xt[1], yt[1], zt[1]])
                    C = np.array([xt[2], yt[2], zt[2]])
                    intersect = ray_triangle_intersect(np.array([0, 0, 0]), ray, A, B, C)
                    print(intersect, i)
                    i += 1
                    if intersect is not None:
                        pass
    print("end")


def generate_rays(resolution, focal_length):
    ray_total_nr = (resolution[0] + 1) * (resolution[1] + 1)
    rays = np.empty([ray_total_nr, 3], dtype=np.float32)
    i = 0
    for x_cord in range(-resolution[0] // 2, resolution[0] // 2 + 1):
        for y_cord in range(-resolution[1] // 2, resolution[1] // 2 + 1):
            ray = np.array([x_cord, y_cord, focal_length])
            rays[i, :] = ray
            i += 1
    return rays


# Ray box intersections

def ray_bounding_box_intersection(b1, b2, l1, l2):
    return True

# Bounding box intersections
