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
# print(drawing_list)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.asarray(drawing_list))
o3d.visualization.draw_geometries([pcd])

alpha = 0.03
print(f"alpha={alpha:.3f}")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

# cv.imshow("Display window", img)
# k = cv.waitKey(0)
