#!/usr/bin/env python
# coding: utf-8


import open3d as o3d
# Read the point cloud
pcd = o3d.io.read_point_cloud('fused.ply')
# Downsample the point cloud
pcd_down = pcd.voxel_down_sample(voxel_size=0.05)
# Remove outliers from the point cloud
pcd_inlier, _ = pcd_down.remove_radius_outlier(nb_points=16, radius=0.1)
# Poisson surface reconstruction
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
 pcd_inlier, depth=9)
# Save the triangle mesh
o3d.io.write_triangle_mesh('meshed.ply', mesh)




