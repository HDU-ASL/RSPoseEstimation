from utils import *
import numpy as np
import cv2
import os
import glob
import math
import time
from tqdm import tqdm, trange
from utils.FilterUtil import *
from utils.pose_utils import *
from utils.PlyUtil import *
import argparse

os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"

def read_directory(directory_name):
    array_of_img = []
    depth_file_list = sorted(glob.glob(os.path.join(directory_name, '*.exr')))
    
    for filename in depth_file_list:
        print(filename)
        img = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
        array_of_img.append(img)

    return array_of_img
        

def FilterNan(points):
    clearPoints = []
    for xyz in points:
        if math.isnan(xyz[0]):
            continue
        if math.isnan(xyz[1]):
            continue
        if math.isnan(xyz[2]):
            continue
        clearPoints.append(xyz)
    return clearPoints


def generateByDepthandCamera(depth_array, camera_pose,scale):
    print("depth_array_size",len(depth_array))
    print("camera_array_size",len(camera_pose))

    assert(len(depth_array)==len(camera_pose))

    resultPoint = []
    with tqdm(total= len(depth_array)) as pbar:
        inner_resultPoint = []
        pbar.set_description('Processing:')

        for depth, pose in zip(depth_array, camera_pose):

            width = len(depth[0])
            height = len(depth)
    
            Z_ = depth[:,:,0]
            Z_ = Z_/scale

            # height cows and width cols
            X = np.zeros((height, width))
            Y = np.zeros((height, width))
    
            for i in range(width):# every cols 
                X[:, i] = np.full(height, i)
            for i in range(height):# every row
                Y[i, :] = np.full(width, i)
            X_ = ((X - c_x ) * Z_) / f_x
            Y_ = ((Y - c_y ) * Z_) / f_y

            cameraPoints = np.array([X_.flatten(), Y_.flatten(), Z_.flatten()])
            
            for index in range(len(cameraPoints[0])):
                camerapoint = [cameraPoints[0][index],cameraPoints[1][index],cameraPoints[2][index]]

                if camerapoint[2] == 0.0:
                    continue
                if np.isnan(camerapoint[0]) or np.isnan(camerapoint[1]) or np.isnan(camerapoint[2]):
                    continue

                
                # blender生成的图片z轴朝内了，需要翻转出去，绕x轴旋转180°
                tranMartix = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
                camerapoint = np.dot(tranMartix, camerapoint)

                transformed_point = np.dot(pose[ :3, :3], camerapoint)
                transformed_point += pose[ :3, 3]

                inner_resultPoint.append(transformed_point)

            
            inner_resultPoint = FilterNan(inner_resultPoint)
            resultPoint.extend(inner_resultPoint)
            pbar.update(1)

    return resultPoint


parser = argparse.ArgumentParser()
parser.add_argument('--path_dir', type=str, 
                    help='the folder include only depth maps')

parser.add_argument('--camera_path', type=str,
                    help='the camera transforms file')

parser.add_argument('--save_path', type=str,
                    help='the saved point cloud save path as ply')


if __name__=='__main__':

    f_x = 1000.0
    f_y =  1000.0
    c_x = 360.0000
    c_y = 240.0000

    args = parser.parse_args()

    path_depth = args.path_dir
    path_camera = args.camera_path
    save_path = args.save_path

    print("path_depth",path_depth)
    print("camera_path",path_camera)

    camera_pose = np.loadtxt(open(path_camera,"rb")) 
    camera_pose = camera_pose.reshape(-1,4,4)

    depth_array = read_directory(path_depth)
    resultPoint = generateByDepthandCamera(depth_array, camera_pose, 1.0)
    
    clearPoints = FilterNan(resultPoint)
    # voxel down sample point cloud, default voxel_size is 0.02
    clearPoints = DownSample(clearPoints)
    clearPoints = np.array(clearPoints)
    print("AfterDownSample",len(clearPoints))
    write_ply(save_path,points=clearPoints)

