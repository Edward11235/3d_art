import cv2 as cv
import sys
import open3d as o3d
import numpy as np

img = cv.imread(cv.samples.findFile("mountain.jpg"))
if img is None:
    sys.exit("Could not read the image.")
gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
shape = gray_img.shape
drawing_list = []
for x in range(shape[0]):
    for y in range(shape[1]):
        drawing_list.append([x, y, gray_img[x, y]])

pcd = o3d.geometry.PointCloud()
whole_pic = []
length = shape[0]
height = shape[1]
for i in range(length):
    temp = []
    for j in range(height):
        temp.append(gray_img[i, j])
    whole_pic.append(temp)

surface_points = []
surface_normal = []

for i in range(1, length - 1):
    for j in range(1, height - 1):
        surface_points.append([i, j, whole_pic[i][j]])
        vec_1 = [1, 0, whole_pic[i][j + 1] - whole_pic[i][j - 1]]
        vec_2 = [0, 1, whole_pic[i + 1][j] - whole_pic[i - 1][j]]
        cross = np.cross(vec_1, vec_2)
        normal = cross / np.linalg.norm(cross)
        surface_normal.append(normal)

print("start creating point cloud")
pcd.points = o3d.utility.Vector3dVector(np.asarray(surface_points))
# pcd.colors = o3d.utility.Vector3dVector(np.asarray(test))
pcd.normals = o3d.utility.Vector3dVector(np.asarray(surface_normal))
distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 3 * avg_dist
# o3d.visualization.draw_geometries([pcd])

print("start mesh")
bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector([radius, radius * 2]))
# poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8, width=0, scale=1.1, linear_fit=False)[0]
o3d.visualization.draw_geometries([pcd, bpa_mesh])
