import argparse
import numpy as np
from utils.PlyUtil import *
from utils.ADD_util import *

parser = argparse.ArgumentParser()
parser.add_argument('--Points', type=str, 
                    help='the path of cursePose')
parser.add_argument('--predictPose', type=str, 
                    help='the path of predictPose')
parser.add_argument('--truePose', type=str, 
                    help='the path of mixPose')
parser.add_argument('--save_result', type=str, 
                    help='the add and adds')

from sklearn.metrics import mean_squared_error
def RMSE( pred_pose, gt_pose):
    pre_R = pred_pose[:, :3]
    pre_t= pred_pose[:, 3:]
    gt_R = gt_pose[:, :3]
    gt_t = gt_pose[:, 3:]
    
    # RMSE计算
    # pre_t = np.mat(pre_t).reshape(3,1)
    # gt_t = np.mat(gt_t).reshape(3,1)
    rmse_t = np.sqrt(mean_squared_error(pre_t, gt_t))
    rmse_R = np.sqrt(mean_squared_error(pre_R, gt_R))
    return rmse_R,rmse_t

if __name__=='__main__':
    args = parser.parse_args()
    path_Points = args.Points
    path_predictPose = args.predictPose
    path_truePose = args.truePose

    truePose = np.genfromtxt(path_truePose, delimiter=' ')
    predictPose = np.genfromtxt(path_predictPose, delimiter=' ')
    Points = load_ply_vtx(path_Points)
    Points = np.asarray(Points)

    # print("truePose",truePose)
    # print("predictPose",predictPose)
    adds,pred_pts, gt_pts = cal_adds_dis(Points, predictPose, truePose)
    add ,pred_pts, gt_pts = cal_add_dis(Points, predictPose, truePose)
    # write_ply("pred_pts.ply",pred_pts)
    # write_ply("gt_pts.ply",gt_pts)
    # rotate_adds = cal_add_dis_rotate(Points, predictPose, truePose)
    
    
    # print("add",add)
    # print("adds",adds)
    # print("rotate_adds",rotate_adds)
    print(adds)
    
    rmse_r,rmse_t = RMSE(predictPose, truePose)

    # print("rmse_r",rmse_r)
    # print("rmse_t",rmse_t)

