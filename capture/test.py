import open3d as o3d
import numpy as np

#Create two random points
randomPoints = np.random.rand(2, 3)
print(randomPoints)
pointSet = o3d.geometry.PointCloud()

pointSet.points = o3d.utility.Vector3dVector(randomPoints)

#Visualize the two random points
o3d.visualization.draw_geometries([pointSet])

#Here I want to add more points to the pointSet
#This solution does not work effective

#Create another random set
p1 = np.random.rand(3, 3)
print(p1)
p2 = np.concatenate((pointSet.points, p1), axis=0)
print(p2)
pointSet2 = o3d.geometry.PointCloud()

pointSet2.points = o3d.utility.Vector3dVector(p2)

o3d.visualization.draw_geometries([pointSet2])