import open3d as o3d


def FindSurface(xyz,distance_threshold=0.03):
    pcd = o3d.geometry.PointCloud() 
    pcd.points = o3d.utility.Vector3dVector(xyz)
    # pcd = o3d.utility.Vector3dVector(xyz)
    # pcd = o3d.geometry
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                         ransac_n=3,
                                         num_iterations=1000)
    [a, b, c, d] = plane_model

    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    
    return plane_model,inliers

def DelPlane(xyz,inliers):
    pcd = o3d.geometry.PointCloud() 
    pcd.points = o3d.utility.Vector3dVector(xyz)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    ans = outlier_cloud.points
    return ans


# ply_path = "a.ply"
# plane_model,inliers = FindSurface(ply_path)
# print(inliers)

# [a, b, c, d] = plane_model

# print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
# # 平面内的点
# pcd = o3d.io.read_point_cloud(ply_path)
# inlier_cloud = pcd.select_by_index(inliers)
# # inlier_cloud.paint_uniform_color([1.0, 0, 0])
# # # 平面外的点
# # outlier_cloud = pcd.select_by_index(inliers, invert=True)
# print(inlier_cloud)
# for i in inlier_cloud:
#     print(i)

# # 可视化
# o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
#                                   zoom=0.8,
#                                   front=[-0.4999, -0.1659, -0.8499],
#                                   lookat=[2.1813, 2.0619, 2.0999],
#                                   up=[0.1204, -0.9852, 0.1215])

# o3d.visualization.draw_geometries([inlier_cloud],
#                                   zoom=0.8,
#                                   front=[-0.4999, -0.1659, -0.8499],
#                                   lookat=[2.1813, 2.0619, 2.0999],
#                                   up=[0.1204, -0.9852, 0.1215])
