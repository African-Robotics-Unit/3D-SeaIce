import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import open3d as o3d
import pyvista as pv
from tools.pcdisplay import show_pointcloud_intensity, plotter_pcdisplay
from rosbags.highlevel import AnyReader
import json
import pandas as pd
import tools.pcprocess as pcprocess
import tools.pctransform as pctransform
import tools.pcio as pcio

CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'p3_config.json')
with open(CONFIG_PATH, "r") as f:
    config = json.load(f)

clouds = pcio.read_clouds("C:/Users/agori/Documents/MATLAB/MSc/bigfiles/pythonOutput/BeforeICP/")
#PerformTform
after_icp_path = config["outputFolder"] + config["preprocessing"]["afterICPpath"]
icp_aln_path = after_icp_path + "ICPtforms.aln"
ICPtforms, labels = pcio.read_aln(icp_aln_path)

clouds_aligned = []
for i, (cloud, H) in enumerate(zip(clouds, ICPtforms)):
    pts = cloud.point["positions"].numpy()        # (N, 3)
    R, t = H[:3, :3], H[:3, 3]
    pts_transformed = (R @ pts.T).T + t

    aligned = o3d.t.geometry.PointCloud()
    aligned.point["positions"] = o3d.core.Tensor(pts_transformed, dtype=o3d.core.Dtype.Float64)
    try:
        aligned.point["intensity"] = cloud.point["intensity"]
    except KeyError:
        pass
    clouds_aligned.append(aligned)

palette = [[1,0,0],[0,1,0],[0,0,1],[1,1,0],[0,1,1],[1,0,1]]
vis = []

for i, cloud in enumerate(clouds_aligned):
    # Convert tensor cloud to legacy for visualization
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(cloud.point["positions"].numpy())
    pc.paint_uniform_color(palette[i % len(palette)])
    vis.append(pc)

o3d.visualization.draw_geometries(vis)

int_min = config["intensityFilter"]["intensityMin"]
int_max = config["intensityFilter"]["intensityMax"]

clouds_intfilt = [pcprocess.filter_intensity_cloud(cloud, int_min, int_max) for cloud in clouds_aligned]

clouds_filtered, clouds_outliers = pcprocess.overlap_filter_optimal(clouds_intfilt)

#Align floor
floor_aln_path = config["outputFolder"] + config["preprocessing"]["beforeICPpath"] + "floor_tforms.aln"
floor_tforms, _ = pcio.read_aln(floor_aln_path)

first_tform = floor_tforms[0]
clouds_adjusted = [pctransform.apply_transform(cloud, first_tform) for cloud in clouds_filtered]

totalFilt=pcprocess.pccat(clouds_adjusted)
total_output_path = config["outputFolder"] + "totalCake_with_planks.ply"
o3d.t.io.write_point_cloud(total_output_path, totalFilt)
show_pointcloud_intensity(totalFilt)
