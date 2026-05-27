import numpy as np
import open3d as o3d
from scipy.signal import find_peaks


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

    # if show_graph:

    #     plt.figure()

    #     plt.plot(z_inc, z_dom)

    #     plt.scatter(
    #         z_inc_max,
    #         dom_max,
    #         c="red",
    #         s=15,
    #         label="max vals"
    #     )

    #     plt.scatter(
    #         z_inc_min,
    #         dom_min,
    #         c="green",
    #         s=15,
    #         label="min vals"
    #     )

    #     plt.text(zfloor, floorval, "floor")

    #     plt.text(zboard, boardval, "pallet")

    #     plt.text(zsurf, valsurf, "surface")

    #     plt.xlabel("Z")

    #     plt.ylabel("Sum abs(Z normals)")

    #     plt.legend()

    #     plt.show()

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