import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import argparse
import gc
import joblib
import json

import numpy as np
import open3d as o3d
import pandas as pd
import pyvista as pv
from pathlib import Path
from rosbags.highlevel import AnyReader

import tools.pcio as pcio
import tools.pcprocess as pcprocess
import tools.pctransform as pctransform
from tools.pcdisplay import show_pointcloud_intensity, show_pointcloud_color, plotter_pcdisplay
from tools.pcread import ros_get_raw, get_min_sz, ros2_get_raw
from tools.roiutils import decode_roi


def main(config_name: str):
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config', f'{config_name}_config.json')
    with open(config_path, "r") as f:
        config = json.load(f)

    pathname = config["preprocessing"]["pathname"]
    source_type = config["preprocessing"]["source_type"]
    #Check ROS1 or ROS2
    if source_type == "ros1":
        all_clouds, fnames = ros_get_raw(pathname)
        minsz, testout = get_min_sz(all_clouds, fnames)

        cloud = pv.PolyData(testout["xyz"])
        cloud["intensity"] = testout["intensity"]
        cloud.save("testout.vtp")
        del cloud
        #plotter1 = plotter_pcdisplay(cloud)

        #Read ROI
        roi = decode_roi(config["preprocessing"]["roi"])

        arr_lidar = pcprocess.crop_merge_downsample(
            roi=roi,
            nclouds=50,
            ds_grid_step=0.01,
            all_clouds=all_clouds,
            start_cloud=9,
        )

        arr_ordered = pctransform.reorder_point_cloud(arr_lidar, config["preprocessing"]["scanOrder"])

        arr_lidar = arr_ordered

        cropped_clouds = [None] * len(arr_lidar)
        cropped_rotated = [None] * len(arr_lidar)
        floor_tforms = [None] * len(arr_lidar)
        #Floor adjustment
        for k in range(len(arr_lidar)):
            # Align floor to horizontal
            floor_align, tform = pctransform.align_floor_tform(arr_lidar[k])

            floor_tforms[k] = tform

            # Inverse transform
            inv_tform = np.linalg.inv(tform)

            # Analyse normal distribution
            sorted_poi, arr_index, pallet_index, floor_index, surf_index = pctransform.norms_analysis(
                floor_align,
                show_graph=False
            )

            z_cutoff = sorted_poi[floor_index + 1] + 0.02

            # Crop floor out in aligned frame
            x_roi = [-np.inf, np.inf]
            y_roi = [-np.inf, np.inf]
            z_roi = [z_cutoff, np.inf]
            roi = [x_roi, y_roi, z_roi]
            cropped_clouds[k] = pcprocess.crop_o3d_t(floor_align, roi)

            # Transform cropped cloud back to original orientation
            cropped_rotated[k] = cropped_clouds[k].clone()
            cropped_rotated[k].transform(inv_tform)
            del floor_align

            #Write floortforms to file
            before_icp_path = config["outputFolder"] + config["preprocessing"]["beforeICPpath"]
            scan_labels = [config["preprocessing"]["fname"] + ch for ch in "ABCDEFGH"[:len(floor_tforms)]]
            pcio.write_aln(before_icp_path + "floor_tforms.aln", floor_tforms, scan_labels)

            #Perform alignment
            coarse_tforms, _ = pcio.read_aln(config["preprocessing"]["coarseALN"])
            arr_coarse = pctransform.apply_rs_tforms(cropped_rotated, coarse_tforms)

            pcio.save_clouds(arr_coarse, before_icp_path, config["preprocessing"]["fname"])
            #show_pointcloud_intensity(cropped_rotated[0])

            del cropped_clouds, cropped_rotated, floor_tforms, arr_lidar
            gc.collect()
    elif source_type == "ros2":
        cache_path = Path(config["outputFolder"]) / "ros2_cache.pkl"
        if cache_path.exists():
            print("Loading from cache...")
            all_clouds, arr_rs_raw, fnames = joblib.load(cache_path)
        else:
            all_clouds, arr_rs_raw, fnames = ros2_get_raw(pathname)
            joblib.dump((all_clouds, arr_rs_raw, fnames), cache_path, compress=0)
            print(f"Saved cache to {cache_path}")

    minsz, testout = get_min_sz(all_clouds, fnames)
    show_pointcloud_color(arr_rs_raw[0])

    #show_pointcloud_intensity(testout)

    # roi = decode_roi(config["preprocessing"]["roi"])

    # arr_lidar = pcprocess.crop_merge_downsample(
    #         roi=roi,
    #         nclouds=config["preprocessing"]["nclouds"],
    #         ds_grid_step=config["preprocessing"]["gridStep"],
    #         all_clouds=all_clouds,
    #         start_cloud=config["preprocessing"]["startCloud"],
    #     )
    


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Preprocess point clouds")
    parser.add_argument("config_name", help="Config prefix, e.g. 'p3' loads p3_config.json")
    args = parser.parse_args()
    main(args.config_name)
