from utils.SurfaceFinder import *
from utils.PlyUtil import *
import argparse
from utils.FilterUtil import *

parser = argparse.ArgumentParser()
parser.add_argument('--load_path', type=str, 
                    help='the path of load point cloud')
parser.add_argument('--save_path', type=str, 
                    help='the path of save point cloud')

if __name__=='__main__':
    args = parser.parse_args()
    load_path = args.load_path
    save_path = args.save_path

    points = load_ply_vtx(load_path)
    points = np.asarray(points)
    points = RmovePointByRadius(points,20,0.05)
    points = RmovePointByRadius(points,100,2.0)

    points = np.asarray(points)
    write_ply(save_path,points)
    