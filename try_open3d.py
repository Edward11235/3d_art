import open3d as o3d
import numpy as np
pcd = o3d.geometry.PointCloud()
test = []
for i in range(10):
    for j in range(10):
        test.append([i, j, i*j])
pcd.points = o3d.utility.Vector3dVector(np.asarray(test))
# pcd.points = o3d.utility.Vector3dVector(np.random.randn(500,3))
o3d.visualization.draw_geometries([pcd])

radii = [0.005, 0.01, 0.02, 0.04]
rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
o3d.visualization.draw_geometries([pcd, rec_mesh])





# distances = pcd.compute_nearest_neighbor_distance()
# avg_dist = np.mean(distances)
# radius = 3 * avg_dist
#
# bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,o3d.utility.DoubleVector([radius, radius * 2]))
# o3d.visualization.draw_geometries([bpa_mesh], mesh_show_back_face=True)
