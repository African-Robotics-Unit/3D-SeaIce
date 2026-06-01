import numpy as np
import open3d as o3d
import pyvista as pv
from pcdisplay import show_pointcloud_intensity, plotter_pcdisplay
from rosbags.highlevel import AnyReader
import json
import pandas as pd
import pctools
import pcalign
import pcanalysis
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy import stats as sp_stats


with open("p3_config.json", "r") as f:
    config = json.load(f)

#Load cloud with planks
cloud_path = config["outputFolder"] + "totalCake_with_planks.ply"
filtered_cloud = pctools.read_cloud(cloud_path)
#show_pointcloud_intensity(filtered_cloud)
#Downsample TO DO
voxel_size = config["geometry"]["downsampleGrid"]

downsampled_cloud = filtered_cloud.voxel_down_sample(voxel_size=voxel_size)
#Perform plank cropping + additional croppping 
# Analyse normal distribution
sorted_poi, arr_index, p1Index, p2Index, p3Index = pctools.norms_analysis(
        downsampled_cloud,
        show_graph=False
    )
z_cutoff = sorted_poi[p3Index + 1] + 0.02 #Add JSON parameterisation
x_roi=[-np.inf, 4.25] #Custom crop
y_roi=[-np.inf, np.inf]
z_roi=[z_cutoff, np.inf]
roi = [x_roi, y_roi, z_roi]
total_cake = pctools.crop_o3d_t(
        downsampled_cloud, roi
    )

#Write total cake
total_output_path = config["outputFolder"] + "totalCake.ply"
o3d.t.io.write_point_cloud(total_output_path, total_cake)

#isolateSurface function
pc_top_filled = pcanalysis.isolate_surface(total_cake) #Fishy...

#InterpolateCloud function
show_pointcloud_intensity(pc_top_filled)

#Stats and display
interpolateScale = config["geometry"]["interpolateGrid"]
F, X, Y = pcanalysis.interpolate_cloud(pc_top_filled, scale=interpolateScale)
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
output = pcanalysis.interest_peaks(F, X, Y, n=n_rows, cutoff=25)

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