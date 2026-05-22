from extractclouds import (
    ros_get_raw,
    get_min_sz,
)
from pathlib import Path
import numpy as np
import open3d as o3d
import pyvista as pv
from pcdisplay import show_pointcloud_intensity
from rosbags.highlevel import AnyReader
import json
import pandas as pd

with open("p3_config.json", "r") as f:
    config = json.load(f)

pathname = config["preprocessing"]["pathname"]

all_clouds, fnames = ros_get_raw(pathname)
minsz, testout = get_min_sz(all_clouds,fnames)

cloud = pv.PolyData(testout["xyz"])
cloud["intensity"] = testout["intensity"]
print("Before saving:", cloud.array_names)
cloud.save("testout.vtp")
show_pointcloud_intensity(cloud)

