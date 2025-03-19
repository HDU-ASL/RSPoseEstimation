import open3d as o3d

# ROR滤波
def RmovePointByRadius(name="",savename=""):

    pcd = o3d.io.read_point_cloud(name)
    # o3d.visualization.draw_geometries([pcd])
    # 半径滤波
    num_points = 20  # 邻域球内的最少点数，低于该值的点为噪声点
    radius = 0.05    # 邻域半径大小
    # 执行半径滤波，返回滤波后的点云ror_pcd和对应的索引ind
    ror_pcd, ind = pcd.remove_radius_outlier(num_points, radius)
    # print("半径滤波后的点云：", ror_pcd)
    # 提取噪声点云
    ror_noise_pcd = pcd.select_by_index(ind,invert = True)
    ror_noise_pcd.paint_uniform_color([1, 0, 1])
    ror_pcd.paint_uniform_color([0, 1, 0])
   
    o3d.io.write_point_cloud(savename,ror_pcd)
    return ror_pcd.points

# ROR滤波
def RmovePointByRadius( xyz, point_nun_threshold = 20, radius_threshold = 0.05 ):
    # 半径滤波
    # num_points 邻域球内的最少点数，低于该值的点为噪声点
    # radius 邻域半径大小
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    # 执行半径滤波，返回滤波后的点云ror_pcd和对应的索引ind
    ror_pcd, ind = pcd.remove_radius_outlier(point_nun_threshold, radius_threshold)
    return ror_pcd.points

# SOR滤波
def StatisticalOutlierRemoval(name="",savename=""):

    pcd = o3d.io.read_point_cloud(name)
    # print('原始点云个数是:',np.array(pcd.points).shape[0])
    # o3d.visualization.draw_geometries([pcd])
    cl,index = pcd.remove_statistical_outlier(nb_neighbors = 100,std_ratio= 2.0)
    new_cloud = pcd.select_by_index(index)

    o3d.io.write_point_cloud(savename,new_cloud)
    return new_cloud.points

# SOR滤波
def StatisticalOutlierRemoval( xyz, nb_neighbors = 100, std_ratio = 2.0):

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    cl,index = pcd.remove_statistical_outlier( nb_neighbors, std_ratio)
    new_cloud = pcd.select_by_index(index)

    return new_cloud.points


def DownSample(name,savename,voxel_size=0.02):
    pcd = o3d.io.read_point_cloud(name)
    downpcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    # o3d.visualization.draw_geometries([downpcd])
    o3d.io.write_point_cloud(savename,downpcd) 
    return downpcd

def DownSample( xyz, voxel_size=0.02):
    pcd = o3d.geometry.PointCloud()
    print("number of xyz:",len(xyz))
    print("voxel_size:",voxel_size)
    pcd.points = o3d.utility.Vector3dVector(xyz)

    downpcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    # o3d.visualization.draw_geometries([downpcd])
    return downpcd.points
