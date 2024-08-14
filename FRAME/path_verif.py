import numpy as np
import open3d as o3d

def read_bin_point_cloud(file_path):
    # Assuming the .bin file contains float32 values in XYZ format
    point_cloud = np.fromfile(file_path, dtype=np.float32)

    if point_cloud.shape[0]%3 == 1:
        point_cloud = np.append(point_cloud, [0,0])

    elif point_cloud.shape[0]%3 == 2:
        point_cloud = np.append(point_cloud, [0])
    
    print(point_cloud.shape)

    point_cloud = point_cloud.reshape(-1, 3)  # Assuming XYZ format
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    return pcd

# pcd = o3d.io.read_point_cloud('/home/prakhar/catkin_ws/src/Firat/FRAME/00/map_1_bin/000000.bin')
pcd = read_bin_point_cloud('/home/prakhar/catkin_ws/src/Firat/FRAME/00/map_1_bin/000002.bin')
print(pcd)
print(np.asarray(pcd.points))
print(np.shape(np.asarray(pcd.points))[0])