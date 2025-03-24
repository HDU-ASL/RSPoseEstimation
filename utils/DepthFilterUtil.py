import numpy as np
import matplotlib.pyplot as plt
from sklearn import mixture
import math


def FilterByDepth( points, pose, cameraIntrinsic, left, right):

    fx, fy, cx, cy = cameraIntrinsic
    
    ans_points = []
    for temp in points:

        transformed_points = np.dot(pose[ :, :3], temp.T)
        transformed_points += pose[ :, 3]

        # 翻个面
        tra = [[1,0,0],[0,-1,0],[0,0,-1]]
        transformed_points = np.dot(np.array(tra),transformed_points.T)

        z = transformed_points[2]
        if z < left or z > right:
            continue

        u = fx * transformed_points[0] / transformed_points[2] + cx
        v = fy * transformed_points[1] / transformed_points[2] + cy

        u = int(u)
        v = int(v)
    
        if u>=640 or u< 0 or v>=480 or v<0:
            # 这点不在当前深度图视角内
            continue

        ans_points.append(temp)

    return ans_points



def FilterByViewReturnDepth( vtx, pose,cameraIntrinsic):

    depth = []#用于生成自己的深度图
    fx, fy, cx, cy = cameraIntrinsic

    for temp in vtx:
    
        transformed_points = np.dot(pose[ :, :3], temp.T)
        transformed_points += pose[ :, 3]

        # 翻个面
        tra = [[1,0,0],[0,-1,0],[0,0,-1]]
        transformed_points = np.dot(np.array(tra),transformed_points.T)

        z = transformed_points[2]

        if False == np.isnan(z):
            if z < 0:# 深度为负，证明在后面,直接扔掉
                continue
        
        u = fx * transformed_points[0] / transformed_points[2] + cx
        v = fy * transformed_points[1] / transformed_points[2] + cy

        u = int(u)
        v = int(v)
    
        if u>=640 or u< 0 or v>=480 or v<0:
            # 这点不在当前深度图视角内
            continue

        if np.isnan(z) == False:
            depth.append(z)
    
    return depth

def isnotnan(n):
    if np.isnan(n):
        return False
    else:
        return True
    
def CalculateMixGaussParam(depth):

    depth = np.array(depth)

    x = np.ravel(depth)
    x = filter(isnotnan,x)
    x = list(x)

    f = np.ravel(x)
    f=f.reshape(-1,1)
    g = mixture.GaussianMixture( n_components=3, covariance_type='full')
    g.fit(f)
    weights = g.weights_
    means = g.means_
    sigmas = g.covariances_

    return means,weights,sigmas

def findMeanFromGaussParam(means,sigmas,threshold = 0.15):
    mid = np.min(means)

    index = 0
    for i in range(len(means)):
        if means[i] == i:
            index = i
    width = sigmas[index]
    if width < 0.1:
        return 0,0
    
    return mid, width


    