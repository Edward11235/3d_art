import numpy as np

whole_pic = []
length = 10
height = 10
for i in range(length):
    for j in range(height):
        whole_pic.append([i, j, i*j])
new_array = np.asarray(whole_pic).reshape([10, 10])
print(new_array)