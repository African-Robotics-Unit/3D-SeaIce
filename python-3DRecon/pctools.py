import numpy as np
import open3d as o3d
from scipy.signal import find_peaks
from scipy.spatial import KDTree
import matplotlib.pyplot as plt


def axang_to_rotm(axis, angle):
    axis = np.asarray(axis, dtype=float)

    norm_axis = np.linalg.norm(axis)

    if np.isclose(norm_axis, 0):
        return np.eye(3)

    axis = axis / norm_axis

    x, y, z = axis

    K = np.array([
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0],
    ])

    R = (
        np.eye(3)
        + np.sin(angle) * K
        + (1 - np.cos(angle)) * (K @ K)
    )

    return R


def rotate_plane(pcd, plane_model):
    """
    Python/Open3D equivalent of MATLAB rotatePlane.

    Parameters
    ----------
    pcd : open3d.geometry.PointCloud
        Input point cloud.

    plane_model : list or ndarray
        Plane coefficients from Open3D:
        [a, b, c, d] for ax + by + cz + d = 0

    Returns
    -------
    pcd_rotated : open3d.geometry.PointCloud
        Transformed point cloud.

    tform : ndarray
        4x4 homogeneous transform.
    """
    plane_model = plane_model.numpy()
    plane_normal = np.asarray(plane_model[:3], dtype=float)

    reference = np.array([0, 0, 1], dtype=float)

    rotation_axis = np.cross(plane_normal, reference)

    rotation_angle = np.arccos(
        np.dot(plane_normal, reference)
        / (np.linalg.norm(plane_normal) * np.linalg.norm(reference))
    )

    rotation_matrix = axang_to_rotm(
        rotation_axis,
        rotation_angle
    )

    trans = np.array([0, 0, 1], dtype=float)

    tform = np.eye(4)
    tform[:3, :3] = rotation_matrix
    tform[:3, 3] = trans

    #pcd_rotated = o3d.geometry.PointCloud(pcd)
    #pcd_rotated.transform(tform)
    pcd_rotated = pcd.clone()
    pcd_rotated.transform(tform)
    return pcd_rotated, tform

def align_floor_tform(pcd):
    """
    Open3D equivalent of MATLAB alignFloorTform.

    Parameters
    ----------
    pcd : open3d.geometry.PointCloud

    Returns
    -------
    pcd_rotated : open3d.geometry.PointCloud
        Rotated point cloud.

    tform : np.ndarray
        4x4 homogeneous transformation matrix.
    """

    max_distance = 0.02
    max_num_trials = 3000
    reference_vector = np.array([0, 0, 1])

    # Fit plane using RANSAC
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=max_distance,
        ransac_n=3,
        num_iterations=max_num_trials,
    )

    [pcd_rotated, tform] = rotate_plane(pcd, plane_model)
    return pcd_rotated, tform

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
            cloud_dict = cur_view[i]

            pcd = dict_to_o3d_t(cloud_dict)

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


def reorder_point_cloud(
    arr_before,
    field_notes,
    target_order="ABCDEFGH",
):
    """
    Reorder point cloud array according to alphabetical field notes.

    Parameters
    ----------
    arr_before : list
        Unordered list of point clouds/views.

    field_notes : str
        Current ordering labels.
        Example: 'HABCDGFE'

    target_order : str
        Desired ordering.
        Default: 'ABCDEFGH'

    Returns
    -------
    arr_after : list
        Reordered point cloud list.

    Example
    -------
    arr_lidar_ordered = reorder_point_cloud(
        arr_lidar,
        'HABCDGFE'
    )
    """

    arr_after = [None] * len(arr_before)

    for i, target_char in enumerate(target_order):

        index = field_notes.index(target_char)

        arr_after[i] = arr_before[index]

    return arr_after

def write_aln(filepath, transforms, labels):
    with open(filepath, "w") as f:
        f.write(f"{len(transforms)}\n")
        for label, tform in zip(labels, transforms):
            f.write(f"# {label}\n")
            for row in tform:
                f.write(" ".join(f"{v:.10f}" for v in row) + "\n")


def norm_levels(pcd, spacing=0.02):
    """
    Python equivalent of MATLAB normLevels().
    """

    # Estimate normals if not already present
    if not pcd.has_normals():
        pcd.estimate_normals()

    points = np.asarray(pcd.points)

    normals = np.asarray(pcd.normals)

    zvals = points[:, 2]

    abs_normals = np.abs(normals)

    znorms = abs_normals[:, 2]

    min_z = np.min(zvals)
    max_z = np.max(zvals)

    z_increments = np.arange(
        min_z,
        max_z + spacing,
        spacing
    )

    percent_dominant_z = np.zeros(len(z_increments))

    for i, z_lower in enumerate(z_increments):

        z_upper = z_lower + spacing

        in_range = (
            (zvals >= z_lower) &
            (zvals < z_upper)
        )

        if np.any(in_range):
            percent_dominant_z[i] = np.sum(
                znorms[in_range]
            )

    return z_increments, percent_dominant_z

def norm_levels_fast(pcd, spacing=0.02):
    if "normals" not in pcd.point:
        pcd.estimate_normals()

    points = pcd.point["positions"].numpy()
    normals = pcd.point["normals"].numpy()

    zvals = points[:, 2]
    znorms = np.abs(normals[:, 2])

    min_z = np.min(zvals)
    max_z = np.max(zvals)

    edges = np.arange(min_z, max_z + spacing, spacing)

    z_dom, _ = np.histogram(
        zvals,
        bins=edges,
        weights=znorms
    )

    z_inc = edges[:-1]

    return z_inc, z_dom

def find_local_max(percent_dominant_z, z_increments):
    """
    Python equivalent of MATLAB findLocalMax().
    """

    peaks, _ = find_peaks(percent_dominant_z)

    peak_vals = percent_dominant_z[peaks]

    peak_z = z_increments[peaks]

    sort_idx = np.argsort(peak_vals)[::-1]

    sorted_local_maxima = peak_vals[sort_idx]

    sorted_z_increments = peak_z[sort_idx]

    return sorted_z_increments, sorted_local_maxima


def find_local_min(percent_dominant_z, z_increments):
    """
    Python equivalent of MATLAB findLocalMin().
    """

    minima, _ = find_peaks(-percent_dominant_z)

    min_vals = percent_dominant_z[minima]

    min_z = z_increments[minima]

    return min_z, min_vals


def norms_analysis(pcd, show_graph=False):
    """
    Python equivalent of MATLAB normsAnalysis().
    """

    z_inc, z_dom = norm_levels_fast(pcd)

    z_inc_max, dom_max = find_local_max(
        z_dom,
        z_inc
    )

    z_inc_min, dom_min = find_local_min(
        z_dom,
        z_inc
    )

    topvals = z_inc_max[:5]
    top_dom = dom_max[:5]

    differences = np.abs(topvals - topvals[0])

    differences[0] = np.inf

    min_index = np.argmin(differences)

    zboard = topvals[min_index]

    zfloor = z_inc_max[0]

    boardval = dom_max[min_index]

    floorval = dom_max[0]

    top_dom = np.delete(top_dom, [0, min_index])

    topvals = np.delete(topvals, [0, min_index])

    zsurf = topvals[0]

    valsurf = top_dom[0]

    if show_graph:

        plt.figure()

        plt.plot(z_inc, z_dom)

        plt.scatter(
            z_inc_max,
            dom_max,
            c="red",
            s=15,
            label="max vals"
        )

        plt.scatter(
            z_inc_min,
            dom_min,
            c="green",
            s=15,
            label="min vals"
        )

        plt.text(zfloor, floorval, "p1")

        plt.text(zboard, boardval, "p2")

        plt.text(zsurf, valsurf, "p3")

        plt.xlabel("Z")

        plt.ylabel("Sum abs(Z normals)")

        plt.legend()

        plt.show()

    poi = np.concatenate([
        z_inc_max,
        z_inc_min
    ])

    sorted_poi = np.sort(poi)

    pallet_index = np.where(
        sorted_poi == zboard
    )[0]

    floor_index = np.where(
        sorted_poi == zfloor
    )[0]

    surf_index = np.where(
        sorted_poi == zsurf
    )[0]

    arr_index = [
        floor_index,
        pallet_index,
        surf_index
    ]

    return (
        sorted_poi,
        arr_index,
        pallet_index,
        floor_index,
        surf_index,
    )

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

    # Downsample all clouds (grid average equivalent)
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

def read_cloud(input_path):
    cloud = o3d.t.io.read_point_cloud(input_path)

    # TensorMap does not support .keys() — check each attribute individually
    found = []
    for attr in ["intensity", "colors", "normals"]:
        try:
            cloud.point[attr]
            found.append(attr)
        except KeyError:
            pass
    
    return cloud

def read_clouds(input_path):
    """
    Read all .ply files in input_path (sorted alphabetically).
    Returns (paths, clouds) where clouds is a list of o3d.t.geometry.PointCloud.
    Intensity and all other attributes are accessible via cloud.point["attribute_name"].
    """
    from pathlib import Path
    import open3d as o3d

    input_path = Path(input_path)
    paths = sorted(input_path.glob("*.ply"))

    if not paths:
        raise FileNotFoundError(f"No .ply files found in {input_path}")

    clouds = []
    for filepath in paths:
        cloud = read_cloud(str(filepath))
        clouds.append(cloud)
    return clouds

