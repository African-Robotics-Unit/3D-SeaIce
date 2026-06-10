from pathlib import Path
import numpy as np
import open3d as o3d
from rosbags.highlevel import AnyReader
import pandas as pd


def get_min_sz(all_clouds, fnames):
    """
    Provides a summary of the number of pointclouds in each bagfile
    and provides the minimum number of frames present in all files.

    Additionally returns a combined test cloud comprising one frame
    from each viewpoint.

    Parameters
    ----------
    all_clouds : list
        Nested list of clouds returned by ros_get_raw()

    fnames : list
        Bag filenames

    Returns
    -------
    minsz : int
        Minimum number of frames across all bag files

    testout : dict
        Combined point cloud made from the first frame
        of each viewpoint
    """

    nviews = len(all_clouds)

    sizes = []

    xyz_list = []
    intensity_list = []

    for k in range(nviews):

        cur = all_clouds[k]

        nclouds = len(cur)

        sizes.append(nclouds)

        # First frame from each bag
        first_cloud = cur[0]

        xyz_list.append(first_cloud.point["positions"].numpy())

        intensity_list.append(first_cloud.point["intensity"].numpy().flatten())

    # Print table summary
    summary = pd.DataFrame({
        "Filename": fnames,
        "NClouds": sizes
    })

    print(summary)

    # Minimum number of frames
    minsz = min(sizes)

    print(f"\nMin size = {minsz}")

    # Concatenate clouds
    xyz_combined = np.vstack(xyz_list)

    intensity_combined = np.concatenate(intensity_list)

    testout = o3d.t.geometry.PointCloud()
    testout.point["positions"] = o3d.core.Tensor(xyz_combined, dtype=o3d.core.Dtype.Float32)
    testout.point["intensity"] = o3d.core.Tensor(intensity_combined[:, None], dtype=o3d.core.Dtype.Float32)

    return minsz, testout


def read_livox_pointcloud2(msg):
    """
    Parse a Livox LiDAR sensor_msgs/PointCloud2 message.

    Applies the same invalid-point filter as ros2GetRawslim.m: drops
    points where XYZ is all-zero and intensity is zero.

    Parameters
    ----------
    msg : sensor_msgs/PointCloud2
        Deserialized ROS2 message from the /livox/lidar topic.

    Returns
    -------
    dict with keys:
        xyz       : ndarray, shape (N, 3), float32
        intensity : ndarray, shape (N,), float32
    """

    dtype = np.dtype({
        "names": ["x", "y", "z", "intensity", "tag", "line"],
        "formats": [
            np.float32,
            np.float32,
            np.float32,
            np.float32,
            np.uint8,
            np.uint8,
        ],
        "offsets": [0, 4, 8, 12, 16, 17],
        "itemsize": 18,
    })

    points = np.frombuffer(msg.data, dtype=dtype)

    xyz = np.column_stack((
        points["x"],
        points["y"],
        points["z"]
    ))

    intensity = points["intensity"]

    valid_points = ~(
        np.all(xyz == 0, axis=1) &
        (intensity == 0)
    )

    xyz = xyz[valid_points]
    intensity = intensity[valid_points]

    pcd = o3d.t.geometry.PointCloud()
    pcd.point["positions"] = o3d.core.Tensor(xyz, dtype=o3d.core.Dtype.Float32)
    pcd.point["intensity"] = o3d.core.Tensor(intensity[:, None], dtype=o3d.core.Dtype.Float32)
    return pcd


def read_realsense_pointcloud2(msg):
    """
    Parse a RealSense sensor_msgs/PointCloud2 message with packed RGB.

    Expects the standard RealSense D-series layout: x, y, z as float32
    at offsets 0/4/8, and a packed RGB float32 at offset 12 (16-byte
    point step). Filters out NaN points and points at z <= 0.

    Parameters
    ----------
    msg : sensor_msgs/PointCloud2
        Deserialized ROS2 message from the /camera/depth/color/points topic.

    Returns
    -------
    dict with keys:
        xyz : ndarray, shape (N, 3), float32  -- RealSense camera frame
        rgb : ndarray, shape (N, 3), uint8
    """

    dtype = np.dtype({
        "names": ["x", "y", "z", "rgb"],
        "formats": [np.float32, np.float32, np.float32, np.float32],
        "offsets": [0, 4, 8, 12],
        "itemsize": msg.point_step,
    })

    points = np.frombuffer(msg.data, dtype=dtype)

    xyz = np.column_stack((points["x"], points["y"], points["z"]))

    packed = points["rgb"].view(np.uint32)
    r = ((packed >> 16) & 0xFF).astype(np.uint8)
    g = ((packed >> 8) & 0xFF).astype(np.uint8)
    b = (packed & 0xFF).astype(np.uint8)
    rgb = np.column_stack([r, g, b])

    valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
    pcd = o3d.t.geometry.PointCloud()
    pcd.point["positions"] = o3d.core.Tensor(xyz[valid], dtype=o3d.core.Dtype.Float32)
    pcd.point["colors"] = o3d.core.Tensor(rgb[valid].astype(np.float32) / 255.0, dtype=o3d.core.Dtype.Float32)
    return pcd


def realsense_to_lidar_frame(pc):
    """
    Rotate a RealSense point cloud into the LiDAR coordinate frame.

    Mirrors realsense_to_LiDAR_ros2.m: X_L = Z_RS, Y_L = -X_RS, Z_L = -Y_RS.

    Parameters
    ----------
    pc : dict
        Point cloud dict with keys xyz (N, 3) and rgb (N, 3).

    Returns
    -------
    dict with keys:
        xyz : ndarray, shape (N, 3), float32  -- LiDAR frame
        rgb : ndarray, shape (N, 3), uint8
    """
    # Mirrors realsense_to_LiDAR_ros2.m: RS Z->X, -RS X->Y, -RS Y->Z
    xyz = pc.point["positions"].numpy()
    xyz_lidar = np.column_stack([xyz[:, 2], -xyz[:, 0], -xyz[:, 1]])
    out = o3d.t.geometry.PointCloud()
    out.point["positions"] = o3d.core.Tensor(xyz_lidar, dtype=o3d.core.Dtype.Float32)
    out.point["colors"] = pc.point["colors"]
    return out


def ros2_get_raw(
    pathname,
    lidar_topic="/livox/lidar",
    rs_topic="/camera/depth/color/points",
    rs_frame_idx=3,
    read_rs=True,
):
    """
    Read all ROS2 bag directories under pathname and extract LiDAR and
    RealSense point clouds. Equivalent to ros2GetRawslim.m.

    Scans for subdirectories whose names start with 'rosbag2', reads
    all frames from lidar_topic, and one RealSense frame (rs_frame_idx)
    from rs_topic per bag. The RealSense cloud is transformed into the
    LiDAR coordinate frame via realsense_to_lidar_frame().

    Parameters
    ----------
    pathname : str or Path
        Directory containing rosbag2* subdirectories.

    lidar_topic : str
        PointCloud2 topic for the Livox LiDAR.

    rs_topic : str
        PointCloud2 topic for the RealSense depth/color stream.

    rs_frame_idx : int
        Index of the RealSense frame to extract per bag (default 3,
        matching MATLAB's {4,1} 1-based indexing).

    Returns
    -------
    all_clouds_lidar : list of list of dict
        Outer list is per bag; inner list is per frame.
        Each dict has keys xyz (N, 3) and intensity (N,).

    arr_rs_raw : list of dict
        One entry per bag. Each dict has keys xyz (N, 3) and rgb (N, 3),
        in LiDAR coordinates.

    fnames : list of str
        Bag directory names in sorted order.
    """

    pathname = Path(pathname)

    bag_dirs = sorted(
        d for d in pathname.iterdir()
        if d.is_dir() and d.name.startswith("rosbag2")
    )

    all_clouds_lidar = []
    arr_rs_raw = []
    fnames = []

    for i, bagdir in enumerate(bag_dirs):
        print(f"[{i+1}/{len(bag_dirs)}] Reading bag: {bagdir.name}")
        fnames.append(bagdir.name)
        lidar_clouds = []
        rs_frames = []

        with AnyReader([bagdir]) as reader:
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == lidar_topic:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    lidar_clouds.append(read_livox_pointcloud2(msg))
                    print(f"  LiDAR frame {len(lidar_clouds)}", end="\r")
                elif read_rs and connection.topic == rs_topic:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    rs_frames.append(read_realsense_pointcloud2(msg))
                    print(f"  RS frame {len(rs_frames)}", end="\r")

        print(f"  Done: {len(lidar_clouds)} LiDAR frames, {len(rs_frames)} RS frames")
        all_clouds_lidar.append(lidar_clouds)
        if read_rs:
            #arr_rs_raw.append(realsense_to_lidar_frame(rs_frames[rs_frame_idx]))
            arr_rs_raw.append(rs_frames[rs_frame_idx])

    return all_clouds_lidar, arr_rs_raw, fnames


def ros_get_raw(pathname, topic="/livox/lidar"):
    """
    Read all ROS1 bag files under pathname and extract LiDAR point clouds.

    Scans for *.bag files and reads every frame from the given topic.
    Use ros2_get_raw() for ROS2 bag directories.

    Parameters
    ----------
    pathname : str or Path
        Directory containing *.bag files.

    topic : str
        PointCloud2 topic for the Livox LiDAR.

    Returns
    -------
    all_clouds : list of list of dict
        Outer list is per bag; inner list is per frame.
        Each dict has keys xyz (N, 3) and intensity (N,).

    fnames : list of str
        Bag filenames in sorted order.
    """

    pathname = Path(pathname)

    bag_files = sorted(pathname.glob("*.bag"))

    all_clouds = []
    fnames = []

    for bagpath in bag_files:

        fnames.append(bagpath.name)

        clouds = []

        with AnyReader([bagpath]) as reader:

            connections = [
                x for x in reader.connections
                if x.topic == topic
            ]

            for connection, timestamp, rawdata in reader.messages(
                connections=connections
            ):

                msg = reader.deserialize(
                    rawdata,
                    connection.msgtype
                )

                cloud = read_livox_pointcloud2(msg)

                clouds.append(cloud)

        all_clouds.append(clouds)

    return all_clouds, fnames
