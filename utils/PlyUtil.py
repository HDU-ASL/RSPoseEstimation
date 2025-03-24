from plyfile import PlyData
import numpy as np

def load_ply_vtx(pth):
    """
    load object vertices
    :param pth: str
    :return: pts: (N, 3)
    """
    # ply就是得到的对象
    ply = PlyData.read(pth)
    # vtx相当于得到的所有的点的信息，每个点有10个属性
    vtx = ply['vertex']
    # 只要有用的3个属性，stack得到的列表是一堆x，一堆y，一堆z,
    # 我们要对其进行转换，axis=-1是在列的维度拼接，
    # 变成一个x，一个y，一个z这种的格式，
    pts = np.stack([vtx['x'], vtx['y'], vtx['z']], axis=-1)
    return pts


from plyfile import PlyData,PlyElement
def write_ply(save_path,points,text=True):
    """
    save_path : path to save: '/yy/XX.ply'
    pt: point_cloud: size (N,3)
    """
    points = [(points[i,0], points[i,1], points[i,2]) for i in range(points.shape[0])]
    vertex = np.array(points, dtype=[('x', np.float32), ('y', np.float32),('z', np.float32)])
    el = PlyElement.describe(vertex, 'vertex', comments=['vertices'])
    PlyData([el], text=text).write(save_path)

