from extractclouds import (
    ros_get_raw,
    get_min_sz,
)
from pathlib import Path
import numpy as np
import open3d as o3d
import pyvista as pv
from pcdisplay import show_pointcloud_intensity, plotter_pcdisplay
from rosbags.highlevel import AnyReader
import json
import pandas as pd
import pctools

#o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
with open("p3_config.json", "r") as f:
    config = json.load(f)

pathname = config["preprocessing"]["pathname"]

all_clouds, fnames = ros_get_raw(pathname)
minsz, testout = get_min_sz(all_clouds,fnames)

cloud = pv.PolyData(testout["xyz"])
cloud["intensity"] = testout["intensity"]
cloud.save("testout.vtp")
del cloud
#plotter1 = plotter_pcdisplay(cloud)

#Read ROI
roi = [
    [0, 7],
    [-5, 5],
    [-np.inf, 2],
]

arr_lidar = pctools.crop_merge_downsample(
    roi=roi,
    nclouds=50,
    ds_grid_step=0.01,
    all_clouds=all_clouds,
    start_cloud=9,
)

#pcd = arr_lidar[0]

#xyz = pcd.point["positions"].numpy()
#intensity = pcd.point["intensity"].numpy().flatten()

#cloudOut = pv.PolyData(xyz)
#cloudOut["intensity"] = intensity

#show_pointcloud_intensity(cloudOut)
#plotter2 = plotter_pcdisplay(cloudOut)

arr_ordered = pctools.reorder_point_cloud(arr_lidar, config["preprocessing"]["scanOrder"])


arr_lidar = arr_ordered

cropped_clouds = [None] * len(arr_lidar)
cropped_rotated = [None] * len(arr_lidar)
floor_tforms = [None] * len(arr_lidar)
#Floor adjustment 
for k in range(len(arr_lidar)):
    # Align floor to horizontal
    floor_align, tform = pctools.align_floor_tform(arr_lidar[k])

    floor_tforms[k] = tform

    # Inverse transform
    inv_tform = np.linalg.inv(tform)

    # Analyse normal distribution
    sorted_poi, arr_index, pallet_index, floor_index, surf_index = pctools.norms_analysis(
        floor_align,
        show_graph=False
    )

    z_cutoff = sorted_poi[floor_index + 1] + 0.02

    # Crop floor out in aligned frame
    x_roi=[-np.inf, np.inf]
    y_roi=[-np.inf, np.inf]
    z_roi=[z_cutoff, np.inf]
    roi = [x_roi, y_roi, z_roi]
    cropped_clouds[k] = pctools.crop_o3d_t(
        floor_align, roi
    )

    # Transform cropped cloud back to original orientation
    cropped_rotated[k] = cropped_clouds[k].clone()
    cropped_rotated[k].transform(inv_tform)
    del floor_align

#Write floortforms to file
scan_labels = [config["preprocessing"]["fname"] + ch for ch in "ABCDEFGH"[:len(floor_tforms)]]
pctools.write_aln("floor_tforms.aln", floor_tforms, scan_labels)

show_pointcloud_intensity(cropped_rotated[0])
del cropped_clouds, cropped_rotated, floor_tforms, arr_lidar
import gc; gc.collect()