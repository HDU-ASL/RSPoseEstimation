from utils.SurfaceFinder import *
from utils.PlyUtil import *
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--load_path', type=str, 
                    help='the path of load point cloud')
parser.add_argument('--save_path', type=str, 
                    help='the path of save point cloud')

parser.add_argument('--height_bottom', type=str, 
                    help='the path of save point cloud')

if __name__=='__main__':
    args = parser.parse_args()
    load_path = args.load_path
    save_path = args.save_path
    height_bottom = float(args.height_bottom)

    ans_points = load_ply_vtx(load_path)
    plane_model,inliers = FindSurface(ans_points,distance_threshold=height_bottom)
    point2 = DelPlane(ans_points,inliers)
    point2 = np.asarray(point2)
    write_ply(save_path,points=point2)