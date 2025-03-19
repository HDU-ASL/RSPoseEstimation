import numpy as np
from sklearn.neighbors import NearestNeighbors
from tqdm import tqdm


def cal_adds_dis(cls_ptsxyz, pred_pose, gt_pose):
    pred_pts = np.dot(pred_pose[:, :3],cls_ptsxyz.copy().T).T + pred_pose[:, 3]
    gt_pts = np.dot(gt_pose[:, :3],cls_ptsxyz.copy().T).T + gt_pose[:, 3]
    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(gt_pts)
    distances, _ = neigh.kneighbors(pred_pts, return_distance=True)
    return np.mean(distances), pred_pts, gt_pts


def cal_add_dis(cls_ptsxyz, pred_pose, gt_pose):
    # 空间距离
    pred_pts = np.dot(pred_pose[:, :3],cls_ptsxyz.copy().T).T + pred_pose[:, 3]
    gt_pts = np.dot(gt_pose[:, :3],cls_ptsxyz.copy().T).T + gt_pose[:, 3]
    mean_dist = np.mean(np.linalg.norm(pred_pts - gt_pts, axis=-1))
    return mean_dist, pred_pts, gt_pts

def adds_all(cls_ptsxyz, pred_pose, gt_pose):
    # 空间距离
    pred_pts = np.dot(pred_pose[:, :3],cls_ptsxyz.copy().T).T + pred_pose[:, 3]
    gt_pts = np.dot(gt_pose[:, :3],cls_ptsxyz.copy().T).T + gt_pose[:, 3]
    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(gt_pts)
    distances, _ = neigh.kneighbors(pred_pts, return_distance=True)
    return distances


def cal_add_dis_rotate(cls_ptsxyz, pred_pose, gt_pose):
    # 空间角度
    # print("pred_pose",pred_pose)
    min_dist = 100000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000

    with tqdm(total=360) as pbar:
        for i in range(0,360):
            pred_pose[:, :3] = np.dot( np.array([[np.cos(np.pi/180*i/360),np.sin(np.pi/180*i/360),0],[-np.sin(np.pi/180*i/360),np.cos(np.pi/180*i/360),0],[0,0,1]]),pred_pose[:, :3])
            # print("pred_pose",pred_pose)
            adds,pred_pts, gt_pts = cal_adds_dis(cls_ptsxyz,pred_pose,gt_pose)
            if adds < min_dist:
                min_dist = adds
            pbar.update(1)
    return min_dist



import numpy as np
from sklearn.neighbors import NearestNeighbors


def chamfer_distance(x, y, metric='l2', direction='bi'):
    """Chamfer distance between two point clouds
    Parameters
    ----------
    x: numpy array [n_points_x, n_dims]
        first point cloud
    y: numpy array [n_points_y, n_dims]
        second point cloud
    metric: string or callable, default ‘l2’
        metric to use for distance computation. Any metric from scikit-learn or scipy.spatial.distance can be used.
    direction: str
        direction of Chamfer distance.
            'y_to_x':  computes average minimal distance from every point in y to x
            'x_to_y':  computes average minimal distance from every point in x to y
            'bi': compute both
    Returns
    -------
    chamfer_dist: float
        computed bidirectional Chamfer distance:
            sum_{x_i \in x}{\min_{y_j \in y}{||x_i-y_j||**2}} + sum_{y_j \in y}{\min_{x_i \in x}{||x_i-y_j||**2}}
    """

    if direction=='y_to_x':
        x_nn = NearestNeighbors(n_neighbors=1, leaf_size=1, algorithm='kd_tree', metric=metric).fit(x)
        min_y_to_x = x_nn.kneighbors(y)[0]
        chamfer_dist = np.mean(min_y_to_x)
    elif direction=='x_to_y':
        y_nn = NearestNeighbors(n_neighbors=1, leaf_size=1, algorithm='kd_tree', metric=metric).fit(y)
        min_x_to_y = y_nn.kneighbors(x)[0]
        chamfer_dist = np.mean(min_x_to_y)
    elif direction=='bi':
        x_nn = NearestNeighbors(n_neighbors=1, leaf_size=1, algorithm='kd_tree', metric=metric).fit(x)
        min_y_to_x = x_nn.kneighbors(y)[0]
        y_nn = NearestNeighbors(n_neighbors=1, leaf_size=1, algorithm='kd_tree', metric=metric).fit(y)
        min_x_to_y = y_nn.kneighbors(x)[0]
        chamfer_dist = np.mean(min_y_to_x) + np.mean(min_x_to_y)
    else:
        raise ValueError("Invalid direction type. Supported types: \'y_x\', \'x_y\', \'bi\'")

    return chamfer_dist


def half_chamfer_distance(pre_points, true_points, metric='l2'):

    true_nn = NearestNeighbors(n_neighbors=1, leaf_size=1, algorithm='kd_tree', metric=metric).fit(true_points)
    min_pre_to_true = true_nn.kneighbors(pre_points)[0]
    half_chamfer_dist = np.mean(min_pre_to_true)
    
    return half_chamfer_dist