import pandas as pd
import open3d as o3d

df = pd.read_csv("cloud_0000.csv")

points = df[["x", "y", "z"]].values

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

o3d.visualization.draw_geometries([pcd])