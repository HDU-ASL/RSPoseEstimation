from utils.PlyUtil import *
import math
import numpy as np
from utils.PC_center_normal_utils import *

def AngleList(MaxScale):
    list_angle = [ ]
    for i in range(MaxScale):
        list_angle.append(2.0 * math.pi / MaxScale * i)
    return list_angle

def Angle2Normal(angle):
    x = math.cos(angle)
    y = math.sin(angle)
    return [x,y,0.0]

def AngleList2NormalList(list_angle):
    NormalList = []
    for angle in list_angle:
        normal = Angle2Normal(angle)
        NormalList.append(normal)

    return NormalList

def normal_list2R_normal_list(list_normal, normal, t):
    list_normal = np.asarray(list_normal)
    R = getT_v2Target([0,0,1],normal)
    # print("R",R)
    # print("list_normal",list_normal)
    R_list_normal = np.dot( R, list_normal.T).T
    # print("T_list_normal",T_list_normal)
    return R_list_normal.tolist()


def cal( points, MaxScale, ScaleNumber,path_pic, center =[0,0,0], normal=[0,0,1]):
    # 依据法线和重心计算密度最大的角度
    # 返回一个角度

    # center 形心
    # normal 旋转法线
    # points 点云
    # MaxScale 将一个360°划分为MaxScale个刻度
    # ScaleMumber 每个扇区应该有ScaleMumber个刻度

    # 从0到360度，划分MaxScale个刻度数量，获得其角度
    list_angle = AngleList(MaxScale)
    # print("list_angle",list_angle)

    # 刻度角转为单位向量
    normalList = AngleList2NormalList(list_angle)
    # print("normalList",normalList)

    # 单位方向旋转至点云位姿上
    R_normal_list = normal_list2R_normal_list(normalList, normal, center)
    # print("T_normal_list",T_normal_list)

    # 计算跨过ScaleNumber的刻度的两个单位向量,用作后续面的法向量
    list_cross_normal = [ ]
    for i in range(MaxScale):
        left = R_normal_list[(i- math.floor(ScaleNumber/2) +MaxScale)%MaxScale]
        right = R_normal_list[(i+ math.ceil(ScaleNumber/2) +MaxScale)%MaxScale]
        list_cross_normal.append([left,right])
    # print("list_cross_normal",list_cross_normal)
        
    # 将点云移动到中心，中心与Oxyz重合
    points = np.asarray(points)
    points = points - center

    # 通过计算面的法向量和原点与坐标点构成的向量的点乘的正负确定在面的哪一面
    countlst = []
    for i in range(MaxScale):
        copypoints = points
        
        left,right = list_cross_normal[i]
        
        res_left = np.dot(left,copypoints.T)
        res_right = np.dot(right,copypoints.T)
        
        count = 0
        for i in range(len(points)):
            if res_left[i] > 0 and res_right[i] < 0:
                count = count + 1
        countlst.append(count)

    # print(countlst)

    np.save('true_angle.npy',countlst)
    
    # 绘制成图
    # import matplotlib.pyplot as plt
    # plt.figure(figsize=(20, 10), dpi=100)
    # plt.plot(countlst)
    # plt.xlabel('Angle(0.1° per number)',fontsize=20)
    # plt.ylabel('Point Number',fontsize=20)
    # plt.xticks(fontsize=20)
    # plt.yticks(fontsize=20)
    # plt.savefig(path_pic)

    # 寻找最大的下标
    index = np.where(countlst==np.max(countlst))
    print("max_index",index)
    max_left, max_right = list_cross_normal[index[0][0]] # 在left面的上面，在right面的下面
    
    # 四边形准备求最大的角度
    toward = np.asarray(max_left)-np.asarray(max_right)
    # 转为单位向量
    vec_norm = np.linalg.norm(toward)
    toward = toward / vec_norm
    print("toward",toward)
    return toward


import numpy as np

import argparse

def TandT(T1,T2):
    # T1左乘T2
    # T1是3行4列的矩阵，T2是3行3列的矩阵
    b = np.array([0,0,0,1])
    T1=np.r_[T1,[b]]

    c = np.array([0,0,0])
    T2 = np.c_[T2,c]
    T2 = np.r_[T2,[b]]

    final = np.dot(T1,T2)
    return final[:3,:]

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # 把密度最大的度数寻转到[1,0,0]，更新到T中

    parser.add_argument('--pose_T', type=str, 
                    default=r"inverse.csv",
                    help='3*4 pose')

    parser.add_argument('--pc', type=str,
                    help='pointcloud name')
    
    parser.add_argument('--pic', type=str,
                    help='where picture saved')
    
    parser.add_argument('--FinalPoints', type=str,
                    default=r"TruePoints.ply",
                    help='the final points')
    
    parser.add_argument('--FinalPose', type=str,
                    default=r"TruePose.csv",
                    help='the last 3*4 pose')
    
    parser.add_argument('--Number', type=str,
                    default=r"12",
                    help='辐数')

    args = parser.parse_args()

    path_point = args.pc
    path_pic = args.pic
    path_FinalPoints = args.FinalPoints
    path_FinalPose = args.FinalPose
    number = int(args.Number)

    
    points = load_ply_vtx(path_point)
    # toward = cal(points , 3600,3600/number, path_pic)
    toward = cal(points , 3600,300, path_pic)

    print("toward",toward)
    
    # 旋转给点云的旋转量
    Rotate_Add2Points = getT_v2Target(toward,[1,0,0])
    # 补充进pose的旋转量
    Rotate_Add2Pose = getT_v2Target([1,0,0],toward)

    points = np.asarray(points)
    print("points",len(points))
    final_points = np.dot(Rotate_Add2Points, points.T).T
    print("final_points",len(final_points))

    path_T = args.pose_T
    pose_T = np.genfromtxt(path_T, delimiter=' ')
    
    final_T = TandT(pose_T,Rotate_Add2Pose)
    print("final_T",final_T)

    np.savetxt( path_FinalPose, final_T, fmt='%f', delimiter=" ")

    write_ply(path_FinalPoints,final_points)
