import open3d as o3d
import numpy as np

pcd = o3d.geometry.PointCloud()
whole_pic = []
length = 10
height = 10
for i in range(length):
    temp = []
    for j in range(height):
        temp.append(i * j)
    whole_pic.append(temp)

surface_points = []
surface_normal = []

for i in range(1, length - 1):
    for j in range(1, length - 1):
        print(i, j)
        surface_points.append([i, j, whole_pic[i][j]])
        vec_1 = [1, 0, whole_pic[i][j+1] - whole_pic[i][j-1]]
        vec_2 = [0, 1, whole_pic[i + 1][j] - whole_pic[i-1][j]]
        cross = np.cross(vec_1, vec_2)
        normal = cross / np.linalg.norm(cross)
        surface_normal.append(normal)


pcd.points = o3d.utility.Vector3dVector(np.asarray(surface_points))
# pcd.colors = o3d.utility.Vector3dVector(np.asarray(test))
pcd.normals = o3d.utility.Vector3dVector(np.asarray(surface_normal))
distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 3 * avg_dist
# o3d.visualization.draw_geometries([pcd])

bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,o3d.utility.DoubleVector([radius, radius * 2]))
poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8, width=0, scale=1.1, linear_fit=False)[0]
o3d.visualization.draw_geometries([pcd, poisson_mesh])