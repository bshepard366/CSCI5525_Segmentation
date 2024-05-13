#!/usr/bin/env python
# coding: utf-8


import numpy as np
import cv2
import csv
import open3d as o3d
data1 = open("text/points3D.txt","r")
data2 = open("text/images.txt","r")
data_3D = data1.readlines()
data_img = data2.readlines()
points_array=[]
rgb_array=[]
img_data_arr = []
for m in data_img:
    str2=m
    str2 = str2.replace("\n","")
    if(str2[-4:] == ".png"):
        str2 = str2.replace(".png"," 123456789")
    arr2 = np.fromstring(str2, dtype=float, sep=" ")
    img_data_arr.append(arr2)



def check_img(img_id,idx):
    flag = False
    rgb=np.array([0,0,0])
    for m in img_data_arr:
        if((int(m[-1]) == 123456789) and (int(m[0]) == img_id)):
            img_name = int(m[-2])
            flag = True
        elif(flag == True):
            index= idx*3
            x=int(m[index])
            y=int(m[index+1])
            name = str(img_name)
            if(len(name)==2):
                name="00"+name+".png"
            elif(len(name)==1):
                name="000"+name+".png"
            elif(len(name)==3):
                name="0"+name+".png"
            name = "semantic_class/"+name
            resp_img = cv2.imread(name,cv2.IMREAD_UNCHANGED)
            rgb = resp_img[y,x]
            flag = False
            break
    return rgb

for l in data_3D:
    str1=l
    str1 = str1.replace("\n","")
    arr1 = np.fromstring(str1, dtype=float, sep=" ")
    point_3D_id = arr1[0]
    color_list=[]
    points_array.append([arr1[1],arr1[2],arr1[3]])
    for i in range(8,arr1.shape[0],2):
        image_id = int(arr1[i])
        point_2d_idx = int(arr1[i+1])
        rgbcolor = check_img(image_id,point_2d_idx)
        color_list.append(rgbcolor)
        
    count = dict()
    for sub_arr in color_list:
        sub_arr_str = str(sub_arr.tolist())
        if sub_arr_str in count:
            count[sub_arr_str] += 1
        else:
            count[sub_arr_str] = 1

    most_freq = max(count, key=count.get)
    most_freq_arr = np.array(eval(most_freq))
    rgb_array.append(most_freq_arr)
    
with open("sparse_3D_points.txt", 'w') as f:
        csv.writer(f, delimiter=' ').writerows(points_array)
        

with open("sparse_rgb.txt", 'w') as f:
        csv.writer(f, delimiter=' ').writerows(rgb_array)




points_array=np.array(points_array)
rgb_array=np.array(rgb_array)
rgb_array=rgb_array/255
points_array=points_array.astype(float)
rgb_array=rgb_array.astype(float)
pc = o3d.geometry.PointCloud()
pc.points = o3d.utility.Vector3dVector(points_array)
pc.colors = o3d.utility.Vector3dVector(rgb_array)
o3d.io.write_point_cloud("Sparse_Semantic_point_cloud.ply", pc)
pcd_inlier, _ = pc.remove_radius_outlier(nb_points=16, radius=1.2)
o3d.io.write_point_cloud("Sparse_Semantic_point_cloud_without_outliers.ply", pcd_inlier)



