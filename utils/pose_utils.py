import numpy as np


def normalize(v):
    """Normalize a vector."""
    return v/np.linalg.norm(v)

def average_poses(poses):
    """
    Calculate the average pose, which is then used to center all poses
    using @center_poses. Its computation is as follows:
    1. Compute the center: the average of pose centers.
    2. Compute the z axis: the normalized average z axis.
    3. Compute axis y': the average y axis.
    4. Compute x' = y' cross product z, then normalize it as the x axis.
    5. Compute the y axis: z cross product x.
    
    Note that at step 3, we cannot directly use y' as y axis since it's
    not necessarily orthogonal to z axis. We need to pass from x to y.

    Inputs:
        poses: (N_images, 3, 4)

    Outputs:
        pose_avg: (3, 4) the average pose
    """
    # 1. Compute the center
    center = poses[..., 3].mean(0) # (3)

    # 2. Compute the z axis
    z = normalize(poses[..., 2].mean(0)) # (3)

    # 3. Compute axis y' (no need to normalize as it's not the final output)
    y_ = poses[..., 1].mean(0) # (3)

    # 4. Compute the x axis
    x = normalize(np.cross(y_, z)) # (3)

    # 5. Compute the y axis (as z and x are normalized, y is already of norm 1)
    y = np.cross(z, x) # (3)

    pose_avg = np.stack([x, y, z, center], 1) # (3, 4)

    return pose_avg

def center_poses(poses):
    """
    Center the poses so that we can use NDC.
    See https://github.com/bmild/nerf/issues/34

    Inputs:
        poses: (N_images, 3, 4)

    Outputs:
        poses_centered: (N_images, 3, 4) the centered poses
        pose_avg: (3, 4) the average pose
    """

    pose_avg = average_poses(poses) # (3, 4)
    pose_avg_homo = np.eye(4)
    pose_avg_homo[:3] = pose_avg # convert to homogeneous coordinate for faster computation
                                 # by simply adding 0, 0, 0, 1 as the last row
    last_row = np.tile(np.array([0, 0, 0, 1]), (len(poses), 1, 1)) # (N_images, 1, 4)
    poses_homo = \
        np.concatenate([poses, last_row], 1) # (N_images, 4, 4) homogeneous coordinate

    poses_centered = np.linalg.inv(pose_avg_homo) @ poses_homo # (N_images, 4, 4)
    poses_centered = poses_centered[:, :3] # (N_images, 3, 4)

    return poses_centered, np.linalg.inv(pose_avg_homo)

# tofu 对位姿求逆
def inverse_poses(poses):

    out_poses = []
    for pose in poses:
        pose = np.pad(pose,((0,1),(0,0)),'constant',constant_values = (0.0)) 
        pose[3][3]=1.0
        pose = inverse_pose(pose)
        # print(pose[:-1])
        out_poses.append(pose[:-1])

    # poses
    # out_poses.
    # a = out_poses[:,0,0]
    # b = out_poses[:,1,0]
    # c = out_poses[:,2,0]
    # d = out_poses[:,0,1]
    # e = out_poses[:,1,1]
    # f = out_poses[:,2,1]
    # g = out_poses[:,0,2]
    # h = out_poses[:,1,2]
    # i = out_poses[:,2,2]
    # x = out_poses[:,0,3]
    # y = out_poses[:,1,3]
    # z = out_poses[:,2,3]
    # out_poses[ :, :3].T
    # out_poses[:,0,3] = -x*a-y*b-z*c
    # out_poses[:,1,3] = -x*d-y*e-z*f
    # out_poses[:,2,3] = -x*g-y*h-z*i

    out_poses = np.array(out_poses)
    
    return out_poses


def inverse_pose(pose):
    out_pose = np.linalg.inv(pose)
    return out_pose