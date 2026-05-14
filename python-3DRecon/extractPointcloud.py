from pathlib import Path
import numpy as np
import open3d as o3d

from rosbags.highlevel import AnyReader

bagpath = Path("2022-07-23-21-18-58.bag")

with AnyReader([bagpath]) as reader:

    connections = [
        x for x in reader.connections
        if x.topic == "/livox/lidar"
    ]

    for connection, timestamp, rawdata in reader.messages(connections=connections):

        msg = reader.deserialize(rawdata, connection.msgtype)

        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
        ])

        points = np.frombuffer(msg.data, dtype=dtype)

        break

xyz = np.column_stack((
    points['x'],
    points['y'],
    points['z']
))

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

pcd = o3d.geometry.PointCloud()

pcd.points = o3d.utility.Vector3dVector(xyz_crop)

axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)

o3d.visualization.draw_geometries([pcd, axes])