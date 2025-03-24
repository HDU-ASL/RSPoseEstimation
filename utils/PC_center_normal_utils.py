import matplotlib.pyplot as plt
import numpy as np
import math
from scipy.optimize import leastsq

def Distance(param,x0,y0,z0):
    a,b,c,d = param
    dis = abs(a*x0+b*y0+c*z0+d)/math.sqrt(a*a+b*b+c*c)
    return dis

# 求重心
def centreOfGravity(points):
    centerPoint=[0,0,0]
    for point in points:
        x,y,z = point
        centerPoint[0]=centerPoint[0]+x
        centerPoint[1]=centerPoint[1]+y
        centerPoint[2]=centerPoint[2]+z
    # centerPoint 重心
    centerPoint[0]=centerPoint[0]/len(points)
    centerPoint[1]=centerPoint[1]/len(points)
    centerPoint[2]=centerPoint[2]/len(points)
    return centerPoint
    
def getNormalPlane(pointlst):
    p0 = [1,1,1,1]
    points = np.array(pointlst,dtype=np.double)

    x = points[:,0]
    y = points[:,1]
    z = points[:,2]

    Para = leastsq(Distance, p0, args=(x,y,z))
    
    # param :[A，B，C，D]
    param = Para[0]
    return param

def getNormalFromPlane(normalPlane):
    # param_withoutD :[A，B，C]
    param_withoutD=np.delete(normalPlane, -1)
    param_withoutD = param_withoutD / np.sqrt(np.sum(param_withoutD**2))
    normal=np.array(param_withoutD)
    return normal
    
def getT_v2z(normal):
    # normal :[A，B，C]
    #求法向量到（0,0,1）的旋转矩阵R_w2c
    vector=normal
    target_vector=np.array([0,0,1])
    c = np.dot(vector, target_vector)
    n_vector = np.cross(vector, target_vector)
    s = np.linalg.norm(n_vector)
    n_vector_invert = np.array((
        [0,-n_vector[2],n_vector[1]],
        [n_vector[2],0,-n_vector[0]],
        [-n_vector[1],n_vector[0],0]
        ))
    I = np.eye(3)
    R_w2c = I + n_vector_invert + np.dot(n_vector_invert, n_vector_invert)/(1+c)
    # print('R_w2c',R_w2c)
    return R_w2c

def getT_v2Target(normal,target_vector):
    # normal :[A，B，C]
    #求法向量到target_vector的旋转矩阵R_w2c
    vector=normal
    target_vector=np.array(target_vector)
    c = np.dot(vector, target_vector)
    n_vector = np.cross(vector, target_vector)
    s = np.linalg.norm(n_vector)
    n_vector_invert = np.array((
        [0,-n_vector[2],n_vector[1]],
        [n_vector[2],0,-n_vector[0]],
        [-n_vector[1],n_vector[0],0]
        ))
    I = np.eye(3)
    R_w2c = I + n_vector_invert + np.dot(n_vector_invert, n_vector_invert)/(1+c)
    # print('R_w2c',R_w2c)
    return R_w2c


