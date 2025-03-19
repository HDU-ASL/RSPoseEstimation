# cursePose * precisePose
import argparse
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('--cursePose', type=str, 
                    help='the path of cursePose')
parser.add_argument('--precisePose', type=str, 
                    help='the path of precisePose')
parser.add_argument('--mixPose', type=str, 
                    help='the path of mixPose')

def TandT(T1,T2):
    # T1左乘T2
    # T1是3行4列的矩阵，T2是3行4列的矩阵
    b = np.array([0,0,0,1])
    T1=np.r_[T1,[b]]
    T2 = np.r_[T2,[b]]

    final = np.dot(T1,T2)
    return final[:3,:]

if __name__=='__main__':
    args = parser.parse_args()
    path_cursePose = args.cursePose
    path_precisePose = args.precisePose
    path_mixPose = args.mixPose

    cursePose = np.genfromtxt(path_cursePose, delimiter=' ')
    precisePose = np.genfromtxt(path_precisePose, delimiter=' ')

    mixPose = TandT(cursePose,precisePose)
    np.savetxt( path_mixPose, mixPose, fmt='%f', delimiter=" ")