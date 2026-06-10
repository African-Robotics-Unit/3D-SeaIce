import numpy as np
import open3d as o3d
from scipy.spatial import KDTree


# --- Format conversion (also in pcio.py — duplicate until cleanup) ---

def dict_to_o3d_t(cloud):
    pcd = o3d.t.geometry.PointCloud()

    pcd.point["positions"] = o3d.core.Tensor(
        cloud["xyz"],
        dtype=o3d.core.Dtype.Float32
    )

    pcd.point["intensity"] = o3d.core.Tensor(
        cloud["intensity"].reshape(-1, 1),
        dtype=o3d.core.Dtype.Float32
    )

    return pcd


# --- ROI crop, merge, downsample (pctools.py) ---

def crop_o3d_t(pcd, roi):
    x_roi, y_roi, z_roi = roi

    pts = pcd.point["positions"]

    dtype = pts.dtype
    device = pts.device

    x_min = o3d.core.Tensor(x_roi[0], dtype=dtype, device=device)
    x_max = o3d.core.Tensor(x_roi[1], dtype=dtype, device=device)

    y_min = o3d.core.Tensor(y_roi[0], dtype=dtype, device=device)
    y_max = o3d.core.Tensor(y_roi[1], dtype=dtype, device=device)

    z_min = o3d.core.Tensor(z_roi[0], dtype=dtype, device=device)
    z_max = o3d.core.Tensor(z_roi[1], dtype=dtype, device=device)

    x = pts[:, 0]
    y = pts[:, 1]
    z = pts[:, 2]

    mask = (
        (x >= x_min) & (x <= x_max) &
        (y >= y_min) & (y <= y_max) &
        (z >= z_min) & (z <= z_max)
    )

    mask = mask.reshape((-1,))

    return pcd.select_by_mask(mask)


def merge_o3d_t(pcd_list):
    positions = np.vstack([
        pcd.point["positions"].numpy()
        for pcd in pcd_list
    ])

    intensities = np.vstack([
        pcd.point["intensity"].numpy()
        for pcd in pcd_list
    ])

    merged = o3d.t.geometry.PointCloud()

    merged.point["positions"] = o3d.core.Tensor(
        positions,
        dtype=o3d.core.Dtype.Float32
    )

    merged.point["intensity"] = o3d.core.Tensor(
        intensities,
        dtype=o3d.core.Dtype.Float32
    )

    return merged


def crop_merge_downsample(
    roi,
    nclouds,
    ds_grid_step,
    all_clouds,
    start_cloud=9,
):
    """
    Open3D tensor version of MATLAB cropMergeDownsample.

    start_cloud=9 gives MATLAB-equivalent startCloud=10.
    """

    processed_clouds = []

    for k, cur_view in enumerate(all_clouds):

        cropped_clouds = []

        end_cloud = start_cloud + nclouds

        for i in range(start_cloud, end_cloud):
            pcd = cur_view[i]

            cropped = crop_o3d_t(pcd, roi)

            cropped_clouds.append(cropped)

        merged = merge_o3d_t(cropped_clouds)

        downsampled = merged.voxel_down_sample(
            voxel_size=ds_grid_step
        )

        processed_clouds.append(downsampled)

        print(
            f"View {k}: "
            f"merged {merged.point['positions'].shape[0]} points → "
            f"downsampled {downsampled.point['positions'].shape[0]} points"
        )

    return processed_clouds


def pccat(clouds):
    """
    Concatenate a list of o3d.t.geometry.PointCloud objects into one,
    equivalent to MATLAB's pccat().
    """
    positions = np.vstack([c.point["positions"].numpy() for c in clouds])

    merged = o3d.t.geometry.PointCloud()
    merged.point["positions"] = o3d.core.Tensor(positions, dtype=o3d.core.Dtype.Float64)

    # Preserve intensity if all clouds have it
    try:
        intensity = np.vstack([c.point["intensity"].numpy() for c in clouds])
        merged.point["intensity"] = o3d.core.Tensor(intensity, dtype=o3d.core.Dtype.Float32)
    except KeyError:
        pass

    # Preserve colors if all clouds have it
    try:
        colors = np.vstack([c.point["colors"].numpy() for c in clouds])
        merged.point["colors"] = o3d.core.Tensor(colors, dtype=o3d.core.Dtype.Float32)
    except KeyError:
        pass

    return merged


def voxel_down_sample_matlab_origin(cloud, voxel_size):
    pts = cloud.point["positions"].numpy()
    origin = pts.min(axis=0)                       # MATLAB anchors here
    shifted = cloud.clone()
    shifted.point["positions"] = o3d.core.Tensor(
        pts - origin, dtype=o3d.core.Dtype.Float64)
    down = shifted.voxel_down_sample(voxel_size)
    down_pts = down.point["positions"].numpy() + origin   # shift back
    down.point["positions"] = o3d.core.Tensor(
        down_pts, dtype=o3d.core.Dtype.Float64)
    return down


def overlap_filter_optimal(clouds, grid_step=0.01, threshold=0.005):
    """
    Filter each cloud to retain only points that overlap with either
    neighbouring cloud (circular: first and last are neighbours).

    Parameters
    ----------
    clouds : list of o3d.t.geometry.PointCloud
    grid_step : float  — voxel size for downsampling before neighbour search
    threshold : float  — max distance to count as overlapping

    Returns
    -------
    filtered  : list of o3d.t.geometry.PointCloud  (overlap points)
    outliers  : list of o3d.t.geometry.PointCloud  (non-overlap points)
    """
    n = len(clouds)

    #arr_down = [c.voxel_down_sample(grid_step) for c in clouds]
    arr_down = [voxel_down_sample_matlab_origin(c, grid_step) for c in clouds]

    filtered, outliers = [], []

    for k in range(n):
        pc1 = arr_down[k]
        pc2 = arr_down[(k + 1) % n]   # next neighbour (wraps to 0)
        pc3 = arr_down[(k - 1) % n]   # prev neighbour (wraps to n-1)

        pts1 = pc1.point["positions"].numpy()
        pts2 = pc2.point["positions"].numpy()
        pts3 = pc3.point["positions"].numpy()

        # KD-tree nearest-neighbour search
        dist2, _ = KDTree(pts2).query(pts1, k=1, workers=-1)
        dist3, _ = KDTree(pts3).query(pts1, k=1, workers=-1)

        valid_mask = (dist2 <= threshold) | (dist3 <= threshold)

        def make_cloud(mask):
            pc = o3d.t.geometry.PointCloud()
            pc.point["positions"] = o3d.core.Tensor(
                pts1[mask], dtype=o3d.core.Dtype.Float64)
            try:
                inten = pc1.point["intensity"].numpy()
                pc.point["intensity"] = o3d.core.Tensor(
                    inten[mask], dtype=o3d.core.Dtype.Float32)
            except KeyError:
                pass
            return pc

        filtered.append(make_cloud(valid_mask))
        outliers.append(make_cloud(~valid_mask))

        n_in  = int(valid_mask.sum())
        n_out = int((~valid_mask).sum())
        print(f"  Cloud {k}: {n_in:,} overlap  |  {n_out:,} outliers")

    return filtered, outliers


def filter_intensity_cloud(pt_cloud, int_min, int_max):
    """
    Filter a point cloud by intensity range.

    Parameters
    ----------
    pt_cloud : o3d.t.geometry.PointCloud
        Input tensor point cloud with an 'intensity' attribute.
    int_min : float
        Minimum intensity value (inclusive).
    int_max : float
        Maximum intensity value (inclusive).

    Returns
    -------
    o3d.t.geometry.PointCloud
        Filtered point cloud containing only points within [int_min, int_max].
    """

    intensity = pt_cloud.point["intensity"].numpy().flatten()

    mask = (intensity >= int_min) & (intensity <= int_max)

    filtered = o3d.t.geometry.PointCloud()
    filtered.point["positions"] = pt_cloud.point["positions"].numpy()[mask]
    filtered.point["intensity"] = intensity[mask].reshape(-1, 1)

    # Preserve any other attributes (e.g. colors, normals)
    for attr in ["colors", "normals"]:
        try:
            filtered.point[attr] = pt_cloud.point[attr].numpy()[mask]
        except KeyError:
            pass

    # Convert back to tensors
    filtered.point["positions"] = o3d.core.Tensor(
        filtered.point["positions"].numpy(), dtype=o3d.core.Dtype.Float64
    )
    filtered.point["intensity"] = o3d.core.Tensor(
        filtered.point["intensity"].numpy(), dtype=o3d.core.Dtype.Float32
    )

    return filtered
