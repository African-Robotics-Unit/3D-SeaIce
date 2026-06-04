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
import tools.pcsurface as pcsurface
import tools.pcmesh as pcmesh
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy import stats as sp_stats
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import pymeshfix
import traceback


def _mesh_face_colors(verts, tris, base_color=(0.8, 0.8, 1.0), light_dir=(0.5, 0.5, 1.0)):
    """Per-face diffuse lighting — approximates MATLAB's Gouraud + headlight."""
    v0, v1, v2 = verts[tris[:,0]], verts[tris[:,1]], verts[tris[:,2]]
    normals = np.cross(v1 - v0, v2 - v0).astype(float)
    norms   = np.linalg.norm(normals, axis=1, keepdims=True)
    normals /= np.where(norms == 0, 1, norms)

    ld      = np.array(light_dir, dtype=float)
    ld     /= np.linalg.norm(ld)
    lighting = 0.3 + 0.7 * np.clip(normals @ ld, 0, 1)   # ambient + diffuse

    return lighting[:, None] * np.array(base_color)

CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'p3_config.json')
with open(CONFIG_PATH, "r") as f:
    config = json.load(f)

#Load cloud with planks
cloud_path = config["outputFolder"] + "totalCake_with_planks.ply"
filtered_cloud = pcio.read_cloud(cloud_path)
#show_pointcloud_intensity(filtered_cloud)
#Downsample TO DO
voxel_size = config["geometry"]["downsampleGrid"]

downsampled_cloud = filtered_cloud.voxel_down_sample(voxel_size=voxel_size)
#Perform plank cropping + additional croppping
# Analyse normal distribution
sorted_poi, arr_index, p1Index, p2Index, p3Index = pctransform.norms_analysis(
        downsampled_cloud,
        show_graph=False
    )
z_cutoff = sorted_poi[p3Index + 1] + 0.02 #Add JSON parameterisation
x_roi=[-np.inf, 4.25] #Custom crop
y_roi=[-np.inf, np.inf]
z_roi=[z_cutoff, np.inf]
roi = [x_roi, y_roi, z_roi]
total_cake = pcprocess.crop_o3d_t(
        downsampled_cloud, roi
    )

#Write total cake
total_output_path = config["outputFolder"] + "totalCake.ply"
o3d.t.io.write_point_cloud(total_output_path, total_cake)

#isolateSurface function
pc_top_filled = pcsurface.isolate_surface(total_cake)

#InterpolateCloud function
#show_pointcloud_intensity(pc_top_filled)

#Stats and display
interpolateScale = config["geometry"]["interpolateGrid"]
showInterpolate = False

F, X, Y = pcsurface.interpolate_cloud(pc_top_filled, scale=interpolateScale)
if showInterpolate:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    surf = ax.plot_surface(X, Y, F,
                        cmap=cm.jet,          # MATLAB default colormap
                        edgecolor='k',         # black grid lines like MATLAB surf
                        linewidth=0.2,
                        antialiased=True)

    fig.colorbar(surf, shrink=0.5, label='Z')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.show()

#Stats
# remove NaN
h = F[~np.isnan(F)]

mean_h   = np.mean(h)
median_h = np.median(h)
std_h    = np.std(h, ddof=1)                          # ddof=1 matches MATLAB std()
mad_h    = np.median(np.abs(h - median_h))            # MAD (unscaled, matches MATLAB mad(h,1))
skew_h   = sp_stats.skew(h)
kurt_h   = sp_stats.kurtosis(h, fisher=False)         # fisher=False matches MATLAB kurtosis()

print("Height Statistics:")
print(f"  Mean:      {mean_h:.4f}")
print(f"  Median:    {median_h:.4f}")
print(f"  STD:       {std_h:.4f}")
print(f"  MAD:       {mad_h:.4f}")
print(f"  Skewness:  {skew_h:.4f}")
print(f"  Kurtosis:  {kurt_h:.4f}")
#InterestPeaks
n_rows, _ = F.shape
output = pcsurface.interest_peaks(F, X, Y, n=n_rows, cutoff=25)

# fig = plt.figure()
# ax  = fig.add_subplot(111, projection='3d')

# surf_h = ax.plot_surface(X, Y, F, cmap=cm.jet, edgecolor='none', alpha=0.9)

# ax.scatter(*output["peaks"].T,        s=10, c='red',    label='Max points')
# ax.scatter(*output["troughs"].T,      s=10, c='blue', label='Trough points')
# ax.scatter(*output["sub_peaks"].T,    s=10, c='green',  label='Sub-peak points')

# # for peak, trough in zip(output["paired_peaks"], output["troughs"]):
# #     ax.plot3D([peak[0], trough[0]], [peak[1], trough[1]], [peak[2], trough[2]],
# #               'k-', linewidth=1.5)

# ax.set_xlabel('X (m)')
# ax.set_ylabel('Y (m)')
# ax.set_zlabel('Z (m)')
# ax.view_init(elev=90, azim=-90)    # equivalent to MATLAB view(2)
# ax.set_aspect('equal')

# fig.colorbar(surf_h, ax=ax, orientation='horizontal', shrink=0.5, label='Z (m)')
# ax.legend(loc='upper right')
# plt.tight_layout()
# plt.show()
interestPeaks = False
if interestPeaks:
    fig, ax = plt.subplots()

    mesh = ax.pcolormesh(X, Y, F, cmap=cm.jet, shading='auto')

    ax.scatter(output["peaks"][:,0],        output["peaks"][:,1],        s=20, c='red',    zorder=3, label='Max points')
    ax.scatter(output["troughs"][:,0],      output["troughs"][:,1],      s=20, c='yellow', zorder=3, label='Trough points')
    ax.scatter(output["sub_peaks"][:,0],    output["sub_peaks"][:,1],    s=10, c='green',  zorder=3, label='Sub-peak points')

    #for peak, trough in zip(output["paired_peaks"], output["troughs"]):
    #    ax.plot([peak[0], trough[0]], [peak[1], trough[1]], 'k-', linewidth=1.5, zorder=2)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_aspect('equal')
    fig.colorbar(mesh, ax=ax, orientation='horizontal', label='Z (m)')
    ax.legend(loc='upper right')
    plt.tight_layout()
    plt.show()

#print("meshing")
#surface_mesh = pcmesh.pc_to_surface_mesh(total_cake, depth=5, density_quantile=0.05)
#surface_mesh = pcmesh.pc_to_surface_mesh_flat_bottom(total_cake, depth=5, density_quantile=-1, bottom_spacing=0.05)
cloud_with_bottom = pcmesh.add_bottom(total_cake, depth=5, density_quantile=-1, bottom_spacing=0.05)
surface_mesh = pcmesh.pc_to_surface_mesh(cloud_with_bottom, depth=5, density_quantile=0.05)
#o3d.visualization.draw_geometries([surface_mesh])
#o3d.io.write_triangle_mesh(config["outputFolder"] + "surfaceMesh.stl", surface_mesh)
#print("mesh done, now fixing the mesh")

#mfix = pcmesh.fix_mesh(surface_mesh)
#mfix_flat= pcmesh.flatten_mesh_bottom(mfix,tolerance_pct=0.05)   # ← post-process
#mfix_flat.mesh.plot(cpos='xy', eye_dome_lighting=True, anti_aliasing=True, smooth_shading=True)
#flat_mesh    = pcmesh.add_flat_bottom(surface_mesh)

# try:
#     flat_mesh = pcmesh.add_flat_bottom(surface_mesh)
# except Exception as e:
#     traceback.print_exc()

# flat_mesh=surface_mesh
# verts    = np.asarray(flat_mesh.vertices)
# tris     = np.asarray(flat_mesh.triangles)
# faces_pv = np.hstack([np.full((len(tris), 1), 3), tris]).ravel()
# pv_mesh  = pv.PolyData(verts, faces_pv)
# pv_mesh.plot(cpos='xy', eye_dome_lighting=True, anti_aliasing=True, smooth_shading=True)

multi_mesh = True
if multi_mesh:
    # Build meshes
    #depths = [4, 5, 6]
    depths = [4]
    #meshes = [pcmesh.pc_to_surface_mesh(total_cake, depth=d, density_quantile=0.05)
            #for d in depths]
    meshes=[surface_mesh]
    # Figure — 17x9 cm matching MATLAB Position
    fig = plt.figure(figsize=(17/2.54, 9/2.54), facecolor='white')

    for k, (mesh, depth) in enumerate(zip(meshes, depths)):
        ax = fig.add_subplot(1, 3, k + 1, projection='3d')

        verts = np.asarray(mesh.vertices)
        tris  = np.asarray(mesh.triangles)

        poly = Poly3DCollection(verts[tris],
                                facecolors=_mesh_face_colors(verts, tris),
                                edgecolors='none')
        ax.add_collection3d(poly)

        ax.set_xlim(verts[:,0].min(), verts[:,0].max())
        ax.set_ylim(verts[:,1].min(), verts[:,1].max())
        ax.set_zlim(verts[:,2].min(), verts[:,2].max())


        ax.set_axis_off()
        ax.view_init(elev=30, azim=45)   # equivalent to view(3)
        ax.set_title(f'Depth {depth}', fontsize=8)

    # TileSpacing='none', Padding='tight' equivalent
    plt.subplots_adjust(wspace=0, hspace=0,
                        left=0.01, right=0.99,
                        top=0.92, bottom=0.01)
    plt.show()
