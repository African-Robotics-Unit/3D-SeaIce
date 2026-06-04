from pathlib import Path
import numpy as np
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

        xyz_list.append(first_cloud["xyz"])

        intensity_list.append(first_cloud["intensity"])

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

    testout = {
        "xyz": xyz_combined,
        "intensity": intensity_combined
    }

    return minsz, testout


def read_livox_pointcloud2(msg):

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

    return {
        "xyz": xyz,
        "intensity": intensity,
    }


def ros_get_raw(pathname, topic="/livox/lidar"):

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
