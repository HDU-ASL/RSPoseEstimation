from utils.SurfaceFinder import *
from utils.PlyUtil import *
import argparse
from utils.FilterUtil import *
from utils.PC_center_normal_utils import *

parser = argparse.ArgumentParser()
parser.add_argument('--load_path', type=str, 
                    help='the path of load point cloud')
parser.add_argument('--save_path', type=str, 
                    help='the path of save point cloud')
parser.add_argument('--save_matrix', type=str, 
                    help='the path of save T matrix')

def calPointinNormal(points,normal,center):
    point = np.asarray(points)
    # 计算出点与向量成正的数量
    points = points.copy() - center
    res_ = np.dot(normal,points.T)
    error_count = 0
    true_count = 0
    for i in range(len(points)):
        if res_[i] > 0:
            true_count = true_count + 1
        else:
            error_count = error_count + 1

    if true_count > error_count:
        return normal
    else:
        return -normal
    

if __name__=='__main__':
    args = parser.parse_args()
    load_path = args.load_path
    save_path = args.save_path
    save_matrix = args.save_matrix

    points = load_ply_vtx(load_path)
    points = np.asarray(points)

    center = centreOfGravity(points)

    # 最小二乘法计算出平面
    normalplane = getNormalPlane(points)
    
    # 计算出单位法线
    normal = getNormalFromPlane(normalplane)
    # 为了消除歧义，取点数多的那一面为正向
    TRUE_normal = calPointinNormal(normal=normal,points=points,center=center)

    print("normal",TRUE_normal)
    # 将单位法线与z轴对齐的逆变换矩阵（注意，重心的偏移另外算）
    R_matrix = getT_v2z(TRUE_normal)
    
    
    
    ans_points = []
    for point in points:
        innerPoint = point - center
        innerPoint = np.dot(R_matrix,innerPoint)
        ans_points.append(innerPoint)

    center = np.asarray(center).T
    R_matrix = np.linalg.inv(R_matrix)
    T_matrix = np.insert(R_matrix,3,center,axis=1)
    print("T_matrix",T_matrix)

    np.savetxt( save_matrix, T_matrix, fmt='%f', delimiter=" ")
    
    ans_points = np.asarray(ans_points)
    write_ply( save_path, ans_points)