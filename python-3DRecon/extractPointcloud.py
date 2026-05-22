from pathlib import Path
import numpy as np
import open3d as o3d
import pyvista as pv
import pandas as pd
from pcdisplay import show_pointcloud_intensity
from rosbags.highlevel import AnyReader
from extractclouds import ros_get_raw
bagpath = Path("C:/Users/agori/Documents/MATLAB/MSc/bigfiles/p3/2022-07-23-21-18-58.bag")

with AnyReader([bagpath]) as reader:

    connections = [
        x for x in reader.connections
        if x.topic == "/livox/lidar"
    ]

    for connection, timestamp, rawdata in reader.messages(connections=connections):

        msg = reader.deserialize(rawdata, connection.msgtype)
        dtype = np.dtype({
            "names": ["x", "y", "z", "intensity", "tag", "line"],
            "formats": [np.float32, np.float32, np.float32, np.float32, np.uint8, np.uint8],
            "offsets": [0, 4, 8, 12, 16, 17],
            "itemsize": 18,
        })

        points = np.frombuffer(msg.data, dtype=dtype)

        break

xyz = np.column_stack((
    points['x'],
    points['y'],
    points['z']
))

intensity = points["intensity"]

x_roi = [-5, 7]
y_roi = [-5, 5]
z_roi = [-5, 2]

mask = (
    (xyz[:,0] >= x_roi[0]) &
    (xyz[:,0] <= x_roi[1]) &

    (xyz[:,1] >= y_roi[0]) &
    (xyz[:,1] <= y_roi[1]) &

    (xyz[:,2] >= z_roi[0]) &
    (xyz[:,2] <= z_roi[1])
)

xyz_crop = xyz[mask]
intensity_crop = intensity[mask]

# Create PyVista point cloud
cloud = pv.PolyData(xyz_crop)
cloud["intensity"] = intensity_crop

show_pointcloud_intensity(cloud)
# Display
# plotter = pv.Plotter()
# plotter.add_mesh(
#     cloud,
#     scalars="intensity",
#     point_size=3,
#     render_points_as_spheres=True,
#     cmap="viridis"
# )

# plotter.add_axes()
# plotter.show_grid()
# plotter.show()


#pcd = o3d.geometry.PointCloud()

#pcd.points = o3d.utility.Vector3dVector(xyz_crop)

#axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)

#o3d.visualization.draw_geometries([pcd, axes])