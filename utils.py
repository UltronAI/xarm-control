import numpy as np
import math
import open3d as o3d

def get_intrinsics_matrix(intrinsics):
    cx, cy, fx, fy = intrinsics.ppx, intrinsics.ppy, intrinsics.fx, intrinsics.fy
    K = np.array(
        [[fx, 0, cx],
         [0, fy, cy],
         [0, 0,   1]]
    )
    return K


def project_camera2world(p_camera, intrinsics):
    # p_camera = np.array([*(pixel_coord * depth), depth])
    K = get_intrinsics_matrix(intrinsics)
    p_world = np.linalg.inv(K) @ p_camera
    return p_world


def get_camera_xyz(depth, x, y, intrinsics):
    cx, cy, fx, fy = intrinsics
    d = np.median(depth[y-1:y+1, x-1:x+1])
    real_x = (x - cx) / fx * d
    real_y = (y - cy) / fy * d
    real_z = np.sqrt(d**2 - real_x**2 - real_y**2)
    xyz = np.array([real_x, real_y, real_z])
    return xyz


def compute_distance(p1, p2):
    return np.sqrt(np.sum((p1 - p2) ** 2))


def quat2matrix(quat):
    w, x, y, z = quat
    R = np.array(
        [[1 - 2 * y**2 - 2 * z**2, 2 * x * y - 2 * z * w,   2 * x * z + 2 * y * w  ],
         [2 * x * y + 2 * z * w,   1 - 2 * x**2 - 2 * z**2, 2 * y * z - 2 * x * w  ],
         [2 * x * z - 2 * y * w,   2 * y * z + 2 * x * w,   1 - 2 * x**2 - 2 * y**2]]
    )
    return R 


def matrix2quat(R):
    w = 0.5 * np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2])
    x = (R[2, 1] - R[1, 2]) / 4 / w 
    y = (R[0, 2] - R[2, 0]) / 4 / w 
    z = (R[1, 0] - R[0, 1]) / 4 / w 
    return np.array([w, x, y, z])


def quat2euler(quat):
    w, x, y, z = quat

    # roll: x
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch: y
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw: z
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    euler = (roll, pitch, yaw)
    angular = []
    for rad in euler:
        angular.append((rad * 180 / math.pi) % 360)

    return angular


def euler2quat(euler, is_rad=False):
    roll, pitch, yaw = euler
    if not is_rad:
        roll = roll / 180 * math.pi
        pitch = pitch / 180 * math.pi
        yaw = yaw / 180 * math.pi
    cy = np.cos(yaw * 0.5) # double cy = cos(yaw * 0.5);
    sy = np.sin(yaw * 0.5) # double sy = sin(yaw * 0.5);
    cp = np.cos(pitch * 0.5) # double cp = cos(pitch * 0.5);
    sp = np.sin(pitch * 0.5) # double sp = sin(pitch * 0.5);
    cr = np.cos(roll * 0.5) # double cr = cos(roll * 0.5);
    sr = np.sin(roll * 0.5) # double sr = sin(roll * 0.5);

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return np.array([w, x, y, z])


def euler2matrix(euler, is_rad=False):
    roll, pitch, yaw = euler
    if not is_rad:
        roll = roll / 180 * math.pi
        pitch = pitch / 180 * math.pi
        yaw = yaw / 180 * math.pi

    Rx = np.array([
        [1., 0.,            0.          ],
        [0., np.cos(roll), -np.sin(roll)],
        [0., np.sin(roll),  np.cos(roll)]
    ])
    Ry = np.array([
        [ np.cos(pitch), 0., np.sin(pitch)],
        [ 0.,            1., 0.           ],
        [-np.sin(pitch), 0., np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0.],
        [np.sin(yaw),  np.cos(yaw), 0.],
        [0.,            0.,          1.]
    ])
    return Rz @ Ry @ Rx


def get_transformation(arm_position):
    x, y, z, roll, pitch, yaw = arm_position
    R = euler2matrix([roll, pitch, yaw])
    t = np.array([x, y, z])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t 
    return T


def convert_depth_frame_to_pointcloud(depth_image, camera_intrinsics ):
    [height, width] = depth_image.shape

    nx = np.linspace(0, width-1, width)
    ny = np.linspace(0, height-1, height)
    u, v = np.meshgrid(nx, ny)
    x = (u.flatten() - camera_intrinsics.ppx)/camera_intrinsics.fx
    y = (v.flatten() - camera_intrinsics.ppy)/camera_intrinsics.fy

    z = depth_image.flatten() #/ 1000;
    x = np.multiply(x,z)
    y = np.multiply(y,z)

    x = x[np.nonzero(z)]
    y = y[np.nonzero(z)]
    z = z[np.nonzero(z)]

    return x, y, z


def get_oriented_bbox(depth, intrinsics):
    point_cloud = convert_depth_frame_to_pointcloud(depth, intrinsics)
    point_cloud = np.stack(point_cloud, axis=-1)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    pcd.remove_statistical_outlier(nb_neighbors=50,
                                   std_ratio=2.0)
    obox = o3d.geometry.OrientedBoundingBox()
    obox = obox.create_from_points(pcd.points)
    return obox.get_center(), obox.get_max_bound(), obox.get_min_bound()

if __name__ == "__main__":
    quat = (-0.699769161734, -0.0484619464362, -0.703755532817, 0.112706300011)
    print(quat2matrix(quat))


