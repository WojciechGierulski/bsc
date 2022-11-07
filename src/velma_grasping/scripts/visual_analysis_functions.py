import time

import numpy as np
import open3d as o3d
import copy
from collections import Counter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from itertools import permutations


def remove_nans(data):
    data = data[np.logical_not(np.isnan(data))]
    return data.reshape((len(data), 1))

def convert_to_numpy_pts_rgb(data):
    shape = (data['x'].shape[0] * data['x'].shape[1], 1)
    x = data['x'].astype(np.float64)
    y = data['y'].astype(np.float64)
    z = data['z'].astype(np.float64)
    r = data['r'].astype(np.float64)
    g = data['g'].astype(np.float64)
    b = data['b'].astype(np.float64)

    x = x.reshape(shape)
    y = y.reshape(shape)
    z = z.reshape(shape)
    r = r.reshape(shape)
    g = g.reshape(shape)
    b = b.reshape(shape)

    nan_arr = np.logical_not(np.isnan(x))
    x = remove_nans(x)
    y = remove_nans(y)
    z = remove_nans(z)
    shape = (len(x), 1)

    r = r[nan_arr].reshape(shape) / 255
    g = g[nan_arr].reshape(shape) / 255
    b = b[nan_arr].reshape(shape) / 255

    return np.hstack((x, y, z)), np.hstack((r, g, b))

def convert_to_o3d(pts, rgb):
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts)
    pc.colors = o3d.utility.Vector3dVector(rgb)
    return pc

def transform_to_world_frame(pc, tf):
    return copy.deepcopy(pc).transform(tf)

def segment_plane(pc):
    plane_model, inliers = pc.segment_plane(distance_threshold=0.0075,
                                            ransac_n=3,
                                            num_iterations=1000)
    return plane_model, inliers


def remove_points_below_plane(pc, plane, offset):
    points = np.array(pc.points)
    rgb = np.array(pc.colors)
    plane[3] -= offset
    good_map = np.dot(np.hstack((points, np.ones((points.shape[0], 1)))), plane) > 0
    pc.points = o3d.utility.Vector3dVector(points[good_map])
    pc.colors = o3d.utility.Vector3dVector(rgb[good_map])
    return pc

def get_table_bbox(inliers, plane, ROIheight, offset):
    # DBScan elements above table in order to find max cluster which is table plane
    inliers = inliers.voxel_down_sample(voxel_size=0.015)
    labels = np.array(inliers.cluster_dbscan(eps=0.1, min_points=100, print_progress=True))
    max_label = labels.max()
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    inliers.colors = o3d.utility.Vector3dVector(colors[:, :3])

    counter = Counter(labels)
    counts = counter.most_common(2)
    if len(counts) == 0:
        return None
    elif len(counts) == 1:
        if counts[0][0] == -1:
            return None
        elif counts[0][0] != -1:
            return counts[0][0]
    elif len(counts) >= 2:
        if counts[0][0] == -1:
            max_class = counts[1][0]
        elif counts[0][0] != 1:
            max_class = counts[0][0]


    # remove points not in main cluster
    good_map = (labels == max_class)
    inliers.points = o3d.utility.Vector3dVector(np.array(inliers.points)[good_map])
    inliers.colors = o3d.utility.Vector3dVector(np.array(inliers.colors)[good_map])

    # Create bounding box above table plane
    plane_normal = np.array([plane[0], plane[1], abs(plane[2])]) # normal vector to plane pointing up
    plane_normal = plane_normal / np.sqrt(np.sum(plane_normal**2)) # normalize vector

    obox = inliers.get_oriented_bounding_box() # bounding box of table plane
    obox.color = (1, 0, 0)
    extent = obox.extent
    center = obox.center

    # Move center up and ROI height / 2
    new_center = center + plane_normal * (ROIheight / 2.0)
    new_extent = copy.deepcopy(extent)
    new_extent[np.argmin(extent)] = ROIheight
    print(center, extent)
    print(new_center, new_extent)
    obox.extent = new_extent
    obox.center = new_center
    return obox


def process_kinect_data(data, tf, tf_calib=None, offset=0.01, ROIheight = 0.5):
    pts, rgb = convert_to_numpy_pts_rgb(data)
    pc = convert_to_o3d(pts, rgb)
    pc = transform_to_world_frame(pc, tf)
    if tf_calib is not None:
        pc = pc.transform(tf_calib)
    plane, inliers = segment_plane(pc)
    inliers = pc.select_by_index(inliers)
    pc = remove_points_below_plane(pc, plane, offset)
    ROIbox = get_table_bbox(inliers, plane, ROIheight, offset)
    pc = pc.crop(ROIbox)
    pc_new, ind = pc.remove_radius_outlier(nb_points=15, radius=0.01)
    labels = np.array(pc_new.cluster_dbscan(eps=0.03, min_points=40, print_progress=True))
    max_label = labels.max()
    n_clusters = max_label + 1
    clusters = []
    cmap = plt.get_cmap('Set2')
    slicedCM = cmap(np.linspace(0, 1, n_clusters))
    for i in range(n_clusters):
        cluster = np.asarray(pc_new.points)[labels == i]
        pc_cluster = o3d.geometry.PointCloud()
        pc_cluster.points = o3d.utility.Vector3dVector(cluster)
        pc_cluster.paint_uniform_color(slicedCM[i,:][0:3])
        clusters.append(pc_cluster)
    return clusters


def classify_pc(cluster, db):
    keys = list(db.objects.keys())
    values = list(db.objects.values())
    scores = []
    transforms = []
    names = []
    #cluster.points = o3d.utility.Vector3dVector(np.asarray(cluster.points) - cluster.get_center())
    o3d.visualization.draw_geometries([cluster])
    for i, (key, value) in enumerate(zip(keys, values)):
        voxel_size = 0.004
        object_pc = value.pc
        source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, object_pc, cluster)
        #o3d.visualization.draw_geometries([object_pc, cluster])
        if not compare_bboxes(source, target): #boxes not similar:
            transforms.append(None)
            scores.append(0)
            names.append(None)
            continue
        start = time.time()
        result_global = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
        print(result_global)
        print("Global: "+ str((time.time()-start)) )
        #draw_registration_result(source_down, target_down, result_global.transformation)
        if result_global.fitness < 0.01:
            transforms.append(None)
            scores.append(0)
            names.append(None)
            continue
        else:
            start = time.time()
            result_local = refine_registration(source, target, voxel_size, result_global.transformation)
            print("Local: " + str(time.time()-start))
            transforms.append(result_local.transformation)
            scores.append(result_local.fitness)
            names.append(value.name)
            print(result_local)
            draw_registration_result(source, target, result_local.transformation)
    max_fitness = max(scores)
    i = scores.index(max_fitness)
    transform = transforms[i]
    name = names[i]
    return transform, max_fitness, name


def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 3
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=20))
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=20))
    pcd.orient_normals_to_align_with_direction(
        orientation_reference=np.array([0., 0., 1.])
    )
    pcd_down.orient_normals_to_align_with_direction(
        orientation_reference=np.array([0., 0., 1.])
    )

    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=30))
    return pcd_down, pcd_fpfh



def prepare_dataset(voxel_size, source, target):
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 2
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def refine_registration(source, target, voxel, init_transform):
    distance_threshold = voxel * 1
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, init_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=9e-01, relative_rmse=1e-09, max_iteration=30))
    return result


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def compare_bboxes(source, target):
    src_box = list(source.get_oriented_bounding_box().extent)
    target_box = list(target.get_oriented_bounding_box().extent)
    src0 = src_box[0]
    src1 = src_box[1]
    src2 = src_box[2]
    perms = np.array(list(permutations(target_box)))
    costs = perms - np.array(src_box).reshape((1,3))
    costs = (costs ** 2).sum(axis=1)
    i = np.argmin(costs)
    trg = perms[i,:]
    trg0 = trg[0]
    trg1 = trg[1]
    trg2 = trg[2]
    if abs(src0-trg0) > 0.05 or abs(src1-trg1) > 0.05 or abs(src2-trg2) > 0.05 or number_at_least_x_times_bigger(src1, src1, 2.5) or number_at_least_x_times_bigger(src0, src0, 2.5) or number_at_least_x_times_bigger(src2, src2, 2.5):
        return False
    return True ## bboxes similar

def number_at_least_x_times_bigger(x1, x2, x):
    if x1 > x2:
        return True if x2*x < x1 else False
    else:
        return True if x1*x < x2 else False
