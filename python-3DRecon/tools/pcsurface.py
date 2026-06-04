import numpy as np
import pandas as pd
import open3d as o3d
from scipy.interpolate import LinearNDInterpolator
from scipy.spatial import ConvexHull, KDTree
from matplotlib.path import Path
from scipy.signal import argrelmax, argrelmin
import matplotlib.pyplot as plt


# --- Private helpers ---

def _get_intensity(cloud):
    """Extract intensity as flat numpy array, or None if not present."""
    try:
        return cloud.point["intensity"].numpy().flatten()
    except KeyError:
        return None


def _set_intensity(cloud, intensity):
    """Write intensity back to a tensor cloud (no-op if None)."""
    if intensity is not None:
        cloud.point["intensity"] = o3d.core.Tensor(
            intensity.reshape(-1, 1).astype(np.float32))


def _estimate_normals(pts, knn=6):
    """Estimate per-point normals using KNN on a legacy Open3D cloud."""
    legacy = o3d.geometry.PointCloud()
    legacy.points = o3d.utility.Vector3dVector(pts)
    legacy.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn))
    return np.asarray(legacy.normals)


# --- Surface extraction pipeline ---

def filter_max_z(cloud, round_factor=0.02):
    """Round X,Y to grid, keep max-Z point (and its intensity) per cell."""
    pts       = cloud.point["positions"].numpy()
    intensity = _get_intensity(cloud)

    # Remove NaN
    valid = ~np.any(np.isnan(pts), axis=1)
    pts   = pts[valid]
    if intensity is not None:
        intensity = intensity[valid]

    # Round X, Y
    pts_r       = pts.copy()
    pts_r[:, 0] = np.round(pts[:, 0] / round_factor) * round_factor
    pts_r[:, 1] = np.round(pts[:, 1] / round_factor) * round_factor

    df = pd.DataFrame(pts_r, columns=["X", "Y", "Z"])
    if intensity is not None:
        df["intensity"] = intensity

    # Index of max-Z row per (X,Y) group
    idx        = df.groupby(["X", "Y"])["Z"].idxmax()
    result_df  = df.loc[idx.values]
    max_z_pts  = result_df[["X", "Y", "Z"]].values

    out = o3d.t.geometry.PointCloud()
    out.point["positions"] = o3d.core.Tensor(max_z_pts, dtype=o3d.core.Dtype.Float64)
    if intensity is not None:
        _set_intensity(out, result_df["intensity"].values)
    return out


def extract_top_cloud(max_z_cloud):
    """Keep only points where Z is the dominant normal direction."""
    pts       = max_z_cloud.point["positions"].numpy()
    intensity = _get_intensity(max_z_cloud)

    normals       = _estimate_normals(pts)
    dominant_dir  = np.argmax(np.abs(normals), axis=1)   # 0=X, 1=Y, 2=Z
    top_mask      = dominant_dir == 2

    out = o3d.t.geometry.PointCloud()
    out.point["positions"] = o3d.core.Tensor(pts[top_mask], dtype=o3d.core.Dtype.Float64)
    if intensity is not None:
        _set_intensity(out, intensity[top_mask])
    return out


def outlier_removal(cloud, k=10, threshold=0.05, show_graph=False):
    """
    Outlier removal based on average K-NN distance.
    Direct port of MATLAB outlierRemoval().

    Points whose average distance to their K nearest neighbours
    is >= threshold are classified as outliers and removed.

    Parameters
    ----------
    cloud     : o3d.t.geometry.PointCloud
    k         : int   — number of nearest neighbours
    threshold : float — avg-distance cutoff (MATLAB hardcodes 0.05)
    show_graph: bool  — show sorted-distance and XY inlier/outlier plots

    Returns
    -------
    o3d.t.geometry.PointCloud — denoised cloud with intensity preserved
    """
    pts       = cloud.point["positions"].numpy()
    intensity = _get_intensity(cloud)

    # K+1 query — first result is the point itself (distance=0), discard it
    dists, _ = KDTree(pts).query(pts, k=k + 1, workers=-1)
    avg_dist = dists[:, 1:].mean(axis=1)        # shape (N,)

    inlier_mask  = avg_dist <  threshold
    outlier_mask = avg_dist >= threshold

    if show_graph:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

        ax1.scatter(np.arange(len(pts)), np.sort(avg_dist), s=15)
        ax1.axhline(threshold, color='red', linestyle='--',
                    label=f'Threshold = {threshold}')
        ax1.set_xlabel('Point index (sorted)')
        ax1.set_ylabel('Avg KNN distance')
        ax1.set_title('Sorted average KNN distances')
        ax1.legend()

        ax2.scatter(pts[inlier_mask,  0], pts[inlier_mask,  1],
                    s=15, c='blue', label=f'Inliers  ({inlier_mask.sum():,})')
        ax2.scatter(pts[outlier_mask, 0], pts[outlier_mask, 1],
                    s=15, c='red',  label=f'Outliers ({outlier_mask.sum():,})')
        ax2.set_xlabel('X'); ax2.set_ylabel('Y')
        ax2.set_aspect('equal')
        ax2.set_title('Inliers vs Outliers (XY)')
        ax2.legend()

        plt.tight_layout()
        plt.show()

    print(f"  Outlier removal: {inlier_mask.sum():,} kept  |  "
          f"{outlier_mask.sum():,} removed")

    out = o3d.t.geometry.PointCloud()
    out.point["positions"] = o3d.core.Tensor(
        pts[inlier_mask], dtype=o3d.core.Dtype.Float64)
    if intensity is not None:
        _set_intensity(out, intensity[inlier_mask])
    return out


def fill_top_cloud(pc_top, max_z_cloud, x_bin_size=0.02):
    """
    Fill surface gaps using non-Z-dominant points from maxZCloud.

    Scans each X-bin of the filtered top surface, and for points in max_z_cloud
    whose normals are not Z-dominant (i.e. vertical walls), injects any that fall
    within the Y-range of that bin. This patches holes where the surface curves
    into near-vertical geometry. Intensity is carried through for both point sets.

    Parameters
    ----------
    pc_top      : o3d.t.geometry.PointCloud — output of extract_top_cloud/outlier_removal
    max_z_cloud : o3d.t.geometry.PointCloud — full max-Z-filtered cloud (from filter_max_z)
    x_bin_size  : float — width of X-bins used for the fill search (metres)

    Returns
    -------
    o3d.t.geometry.PointCloud — combined cloud; blue=original top, red=filled points
    """
    all_pts       = max_z_cloud.point["positions"].numpy()
    all_intensity = _get_intensity(max_z_cloud)
    has_intensity = all_intensity is not None

    # Normals + dominant direction on maxZCloud
    normals      = _estimate_normals(all_pts)
    dominant_dir = np.argmax(np.abs(normals), axis=1)

    # Non-top points with Z > 0.1
    non_top_mask      = dominant_dir != 2
    non_top_pts       = all_pts[non_top_mask]
    non_top_intensity = all_intensity[non_top_mask] if has_intensity else None

    z_filter          = non_top_pts[:, 2] > 0.1
    non_top_pts       = non_top_pts[z_filter]
    if has_intensity:
        non_top_intensity = non_top_intensity[z_filter]

    filtered_pts       = pc_top.point["positions"].numpy()
    filtered_intensity = _get_intensity(pc_top) if has_intensity else None

    x_min, x_max = filtered_pts[:, 0].min(), filtered_pts[:, 0].max()
    x_edges      = np.arange(x_min, x_max + x_bin_size, x_bin_size)

    added_pts = []
    added_int = []

    for i in range(len(x_edges) - 1):
        x_lo, x_hi = x_edges[i], x_edges[i + 1]

        in_bin     = (filtered_pts[:, 0] >= x_lo) & (filtered_pts[:, 0] < x_hi)
        slice_pts  = filtered_pts[in_bin]
        if len(slice_pts) == 0:
            continue

        y_min, y_max = slice_pts[:, 1].min(), slice_pts[:, 1].max()

        in_x  = (non_top_pts[:, 0] >= x_lo) & (non_top_pts[:, 0] < x_hi)
        in_y  = (non_top_pts[:, 1] >= y_min) & (non_top_pts[:, 1] <= y_max)
        mask  = in_x & in_y
        if mask.any():
            added_pts.append(non_top_pts[mask])
            if has_intensity:
                added_int.append(non_top_intensity[mask])

    added_pts_arr = np.vstack(added_pts) if added_pts else np.empty((0, 3))
    added_int_arr = np.concatenate(added_int) if (has_intensity and added_int) \
                    else (np.empty(0) if has_intensity else None)

    combined_pts = np.vstack([filtered_pts, added_pts_arr])
    colors = np.vstack([
        np.tile([0.0, 0.0, 1.0], (len(filtered_pts),    1)),  # blue — top
        np.tile([1.0, 0.0, 0.0], (len(added_pts_arr),   1)),  # red  — fill
    ])

    out = o3d.t.geometry.PointCloud()
    out.point["positions"] = o3d.core.Tensor(combined_pts, dtype=o3d.core.Dtype.Float64)
    out.point["colors"]    = o3d.core.Tensor(colors.astype(np.float32))
    if has_intensity:
        combined_intensity = np.concatenate([filtered_intensity, added_int_arr])
        _set_intensity(out, combined_intensity)
    return out


def isolate_surface(pc_total_cake):
    """
    Full pipeline to extract the top surface of a sea-ice point cloud.

    Runs filter_max_z -> extract_top_cloud -> outlier_removal -> fill_top_cloud
    in sequence, returning a cleaned and gap-filled surface cloud.

    Parameters
    ----------
    pc_total_cake : o3d.t.geometry.PointCloud — raw full-volume scan

    Returns
    -------
    o3d.t.geometry.PointCloud — isolated top surface, ready for interpolation
    """
    max_z_cloud     = filter_max_z(pc_total_cake,  round_factor=0.02)
    pc_top          = extract_top_cloud(max_z_cloud)
    pc_top_filtered = outlier_removal(pc_top, k=10, show_graph=False)
    pc_top_filled   = fill_top_cloud(pc_top_filtered, max_z_cloud, x_bin_size=0.02)
    return pc_top_filled


# --- Interpolation and peak detection ---

def interpolate_cloud(cloud, scale):
    """
    Interpolate Z values of a point cloud onto a regular grid,
    masked to the convex hull of the input points.

    Equivalent to MATLAB interpolateCloud using scatteredInterpolant('linear').

    Parameters
    ----------
    cloud : o3d.t.geometry.PointCloud
    scale : float  — grid spacing (same as MATLAB 'scale')

    Returns
    -------
    F : np.ndarray (ny, nx)  — interpolated Z, NaN outside convex hull
    X : np.ndarray (ny, nx)  — meshgrid X coordinates
    Y : np.ndarray (ny, nx)  — meshgrid Y coordinates
    """
    pts = cloud.point["positions"].numpy()
    x = pts[:, 0].astype(np.float64)
    y = pts[:, 1].astype(np.float64)
    f = pts[:, 2].astype(np.float64)

    # Mirror MATLAB: linspace(min+scale, max-scale, (range/scale)+2)
    n_x = int(round((x.max() - x.min()) / scale)) + 2
    n_y = int(round((y.max() - y.min()) / scale)) + 2
    xlin = np.linspace(x.min() + scale, x.max() - scale, n_x)
    ylin = np.linspace(y.min() + scale, y.max() - scale, n_y)

    X, Y = np.meshgrid(xlin, ylin)

    # Scattered linear interpolation — equivalent to scatteredInterpolant(...,'linear')
    interp = LinearNDInterpolator(np.column_stack([x, y]), f)
    F = interp(X, Y)   # already NaN outside Delaunay triangulation

    # Convex hull mask — equivalent to delaunayTriangulation + convexHull + inpolygon
    hull     = ConvexHull(np.column_stack([x, y]))            # vertices in CCW order (2D)
    hull_pts = np.column_stack([x, y])[hull.vertices]
    hull_path = Path(hull_pts, closed=True)

    grid_pts = np.column_stack([X.ravel(), Y.ravel()])
    mask = hull_path.contains_points(grid_pts, radius=1e-9)   # radius>0 includes boundary
    mask = mask.reshape(X.shape)

    F[~mask] = np.nan

    return F, X, Y


def interest_peaks(F, X, Y, n, cutoff):
    """
    Detect surface peaks and troughs using the Rayleigh criterion.

    For each row of F, finds the dominant local maximum, then searches
    leftward for a qualifying trough (Z-drop >= cutoff%).

    Parameters
    ----------
    F      : (nRows, nCols) ndarray — interpolated Z, NaN outside hull
    X, Y   : (nRows, nCols) ndarray — meshgrid coordinates
    n      : int   — number of rows to process
    cutoff : float — minimum % drop from peak to qualify a trough

    Returns
    -------
    dict with keys:
        peaks        — (N, 3) dominant maximum per row
        troughs      — (M, 3) qualifying troughs
        paired_peaks — (M, 3) dominant peaks paired 1:1 with troughs
        sub_peaks    — (M, 3) strongest local max left of each trough
        avg_angle    — float, mean X-Z slope angle in degrees
    """
    peaks       = np.zeros((n, 3))
    troughs     = []
    paired_peaks = []
    sub_peaks   = []
    n_peaks     = 0

    for i in range(n):
        row      = F[i, :]
        valid    = ~np.isnan(row)
        orig_idx = np.where(valid)[0]
        z        = row[valid]

        if len(z) < 3:
            continue

        max_idx = argrelmax(z, order=1)[0]
        min_idx = set(argrelmin(z, order=1)[0])

        if len(max_idx) == 0:
            continue

        # Dominant maximum for this row
        top_idx = max_idx[np.argsort(z[max_idx])[::-1][0]]
        top_z   = z[top_idx]

        peaks[n_peaks] = [X[i, orig_idx[top_idx]], Y[i, orig_idx[top_idx]], top_z]
        n_peaks += 1

        # Search leftward for qualifying trough
        idx_found = None
        for k in range(top_idx, -1, -1):
            if k in min_idx and (top_z - z[k]) / top_z * 100 >= cutoff:
                idx_found = k
                break

        if idx_found is None:
            continue

        troughs.append     ([X[i, orig_idx[idx_found]], Y[i, orig_idx[idx_found]], z[idx_found]])
        paired_peaks.append([X[i, orig_idx[top_idx]],   Y[i, orig_idx[top_idx]],   top_z       ])

        # Strongest local max to the left of the trough
        left_max = max_idx[max_idx <= idx_found]
        if len(left_max) > 0:
            best = left_max[np.argmax(z[left_max])]
            sub_peaks.append([X[i, orig_idx[best]], Y[i, orig_idx[best]], z[best]])

    peaks        = peaks[:n_peaks]
    troughs      = np.array(troughs)      if troughs      else np.empty((0, 3))
    paired_peaks = np.array(paired_peaks) if paired_peaks else np.empty((0, 3))
    sub_peaks    = np.array(sub_peaks)    if sub_peaks    else np.empty((0, 3))

    # Slope angle per peak-trough pair
    if len(paired_peaks) > 0:
        dz     = paired_peaks[:, 2] - troughs[:, 2]
        dx     = paired_peaks[:, 0] - troughs[:, 0]
        angles = np.degrees(np.arctan2(dz, dx))
    else:
        angles = np.array([])

    return {
        "peaks"        : peaks,
        "troughs"      : troughs,
        "paired_peaks" : paired_peaks,
        "sub_peaks"    : sub_peaks,
        "avg_angle"    : float(np.nanmean(angles)),
    }
