import numpy as np
import pandas as pd
import open3d as o3d
from scipy.interpolate import LinearNDInterpolator
from scipy.spatial import ConvexHull
from matplotlib.path import Path
from scipy.signal import argrelmax, argrelmin
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy.spatial import Delaunay
import contextlib, os
import pymeshfix
import trimesh
from shapely.geometry import Polygon
import pyvista as pv
from scipy.spatial import KDTree
@contextlib.contextmanager
def suppress_cpp_output():
    """Redirect C++ stdout/stderr to devnull — for Open3D Poisson warnings."""
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_stderr = os.dup(2)
    old_stdout = os.dup(1)
    try:
        os.dup2(devnull, 2)
        os.dup2(devnull, 1)
        yield
    finally:
        os.dup2(old_stderr, 2)
        os.dup2(old_stdout, 1)
        os.close(devnull)
        os.close(old_stderr)
        os.close(old_stdout)

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


from scipy.spatial import KDTree
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


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
    pc_top_filtered = outlier_removal(pc_top,k=10, show_graph=False)
    pc_top_filled   = fill_top_cloud(pc_top_filtered, max_z_cloud, x_bin_size=0.02)
    return pc_top_filled

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


def pc_to_surface_mesh(cloud, depth=9, density_quantile=0.05):
    """
    Poisson surface reconstruction from a point cloud.
    Equivalent to MATLAB pc2surfaceMesh(cloud, 'poisson').

    Parameters
    ----------
    cloud            : o3d.t.geometry.PointCloud
    depth            : int   — octree depth (higher = finer mesh, slower)
                               typical range: 8–11
    density_quantile : float — remove triangles below this density quantile
                               to trim reconstruction artefacts (0 = keep all)

    Returns
    -------
    mesh : o3d.geometry.TriangleMesh
    """
    # Convert tensor cloud to legacy (Poisson requires legacy API)
    pts = cloud.point["positions"].numpy()
    legacy = o3d.geometry.PointCloud()
    legacy.points = o3d.utility.Vector3dVector(pts)
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

    # Normals are required for Poisson — estimate and orient consistently
    legacy.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
    legacy.orient_normals_consistent_tangent_plane(k=15)

    #with suppress_cpp_output():
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(legacy, depth=depth)

    # Trim low-density vertices (artefacts outside the true surface)
    if density_quantile > 0:
        densities = np.asarray(densities)
        remove = densities < np.quantile(densities, density_quantile)
        mesh.remove_vertices_by_mask(remove)

    hull = Delaunay(pts)
    verts   = np.asarray(mesh.vertices)
    outside = hull.find_simplex(verts) < 0
    mesh.remove_vertices_by_mask(outside)
    mesh.compute_vertex_normals()
    # hull  = Delaunay(pts)
    # verts = np.asarray(mesh.vertices)

    # outside = hull.find_simplex(verts) < 0
    # z_min   = pts[:, 2].min()

    # # Flatten outside vertices to z_min instead of removing them
    # verts[outside, 2] = z_min

    # mesh.vertices = o3d.utility.Vector3dVector(verts)
    # mesh.compute_vertex_normals()
    return mesh



def fix_mesh(mesh_o3d, z_floor=0.0):
    """
    Repair mesh topology using MeshFix (Netfabb algorithm).

    Parameters
    ----------
    mesh_o3d : o3d.geometry.TriangleMesh — potentially non-manifold input mesh
    z_floor  : float — unused, reserved for future translation to print bed

    Returns
    -------
    pymeshfix.MeshFix — repaired mesh (access vertices via .v, faces via .f)
    """
    verts = np.asarray(mesh_o3d.vertices)
    tris  = np.asarray(mesh_o3d.triangles)

    # Step 1 — MeshFix (identical to Netfabb's repair algorithm)
    mf = pymeshfix.MeshFix(verts, tris)
    mf.repair(verbose=False)

    #tm = trimesh.Trimesh(vertices=mf.v, faces=mf.f)

    # Step 2 — Translate so bottom sits at z_floor (print bed)
    #tm.apply_translation([0, 0, z_floor - float(mf.v[:, 2].min())])

    return mf

def flatten_mesh_bottom(mf, tolerance_pct=0.01):
    """
    Snap near-bottom vertices to z_min, creating a flat base.
    
    Parameters
    ----------
    mf            : pymeshfix.MeshFix object
    tolerance_pct : float — fraction of total mesh height to snap
                    (default 1% catches small bottom bumps from MeshFix repair)

    Returns
    -------
    pymeshfix.MeshFix — mesh with snapped flat base
    """
    verts = mf.v.copy()
    z_min = verts[:, 2].min()
    z_max = verts[:, 2].max()
    tolerance = (z_max - z_min) * tolerance_pct

    bottom_mask = verts[:, 2] <= z_min + tolerance
    verts[bottom_mask, 2] = z_min

    return pymeshfix.MeshFix(verts, mf.f)
def _boundary_loops(mesh):
    """
    Find ordered boundary vertex loops in a trimesh mesh.

    Boundary edges appear in exactly one face. Builds an adjacency graph over
    those edges and walks connected loops. Replaces trimesh.graph.boundary_loops,
    which was removed in newer trimesh versions.

    Parameters
    ----------
    mesh : trimesh.Trimesh

    Returns
    -------
    list of np.ndarray — each array is an ordered sequence of vertex indices
                         forming one closed boundary loop
    """
    # Boundary edges appear in exactly one face
    edges = mesh.edges_sorted
    unique, counts = np.unique(edges, axis=0, return_counts=True)
    b_edges = unique[counts == 1]

    if len(b_edges) == 0:
        return []

    # Build adjacency for boundary vertices only
    adj = {}
    for v0, v1 in b_edges:
        adj.setdefault(int(v0), []).append(int(v1))
        adj.setdefault(int(v1), []).append(int(v0))

    # Walk each loop
    loops, visited = [], set()
    for start in list(adj.keys()):
        if start in visited:
            continue
        loop = [start]; visited.add(start); cur = start
        while True:
            nxt = next((v for v in adj[cur] if v not in visited), None)
            if nxt is None:
                break
            loop.append(nxt); visited.add(nxt); cur = nxt
        if len(loop) > 2:
            loops.append(np.array(loop))

    return loops
def add_flat_bottom(mesh_o3d, z_floor=None):
    """
    Close an open surface mesh with a flat polygon bottom.

    Designed for CFD use — replaces MeshFix for objects where the underside
    is unknown (e.g. pancake ice on deck). Produces a watertight solid with
    a geometrically clean flat base suitable for CFD boundary conditions.

    Parameters
    ----------
    mesh_o3d : o3d.geometry.TriangleMesh — open surface (from pc_to_surface_mesh)
    z_floor  : float — Z height of flat base (default: mesh z_min)

    Returns
    -------
    o3d.geometry.TriangleMesh — watertight mesh with flat bottom
    """

    verts = np.asarray(mesh_o3d.vertices)
    tris  = np.asarray(mesh_o3d.triangles)

    tm = trimesh.Trimesh(vertices=verts, faces=tris, process=False)
    trimesh.repair.fix_winding(tm)
    trimesh.repair.fix_normals(tm)

    if z_floor is None:
        z_floor = float(verts[:, 2].min())

    loops = _boundary_loops(tm)
    if not loops:
        print("Mesh already watertight — no boundary found.")
        return mesh_o3d

    # Largest loop = bottom boundary
    main_loop = max(loops, key=len)
    n         = len(main_loop)
    top_pts   = verts[main_loop]
    bot_pts   = top_pts.copy()
    bot_pts[:, 2] = z_floor

    patches = [tm]

    # --- Vertical side walls: connect boundary loop down to z_floor ---
    side_verts = np.vstack([top_pts, bot_pts])
    side_faces = []
    for i in range(n):
        j = (i + 1) % n
        side_faces += [[i, j, i + n],
                       [j, j + n, i + n]]
    patches.append(trimesh.Trimesh(
        vertices=side_verts,
        faces=np.array(side_faces),
        process=False))

    # --- Flat bottom cap: single polygon triangulated with earcut ---
    poly           = Polygon(bot_pts[:, :2])
    cap_v2d, cap_f = trimesh.creation.triangulate_polygon(poly, engine='earcut')
    cap_verts      = np.column_stack([cap_v2d, np.full(len(cap_v2d), z_floor)])
    patches.append(trimesh.Trimesh(vertices=cap_verts, faces=cap_f, process=False))

    solid = trimesh.util.concatenate(patches)
    solid.merge_vertices()
    #solid.remove_duplicate_faces()
    solid.process() 
    trimesh.repair.fix_winding(solid)
    trimesh.repair.fix_normals(solid)

    out = o3d.geometry.TriangleMesh()
    out.vertices  = o3d.utility.Vector3dVector(np.array(solid.vertices))
    out.triangles = o3d.utility.Vector3iVector(np.array(solid.faces))
    out.compute_vertex_normals()
    return out

def pc_to_surface_mesh_flat_bottom(cloud, depth=9, density_quantile=0.05,
                                    z_floor=None, bottom_spacing=None):
    """
    Poisson reconstruction with a synthetic plane of points added at z_floor
    before reconstruction. Poisson naturally incorporates a flat bottom
    without any post-processing mesh surgery.

    Parameters
    ----------
    cloud           : o3d.t.geometry.PointCloud
    depth           : int   — Poisson octree depth
    density_quantile: float — density pruning threshold
    z_floor         : float — Z height of flat bottom (default: cloud z_min)
    bottom_spacing  : float — grid spacing for synthetic bottom points
                              (default: matches cloud average point spacing)

    Returns
    -------
    o3d.geometry.TriangleMesh — Poisson mesh with a flat bottom face,
                                cropped to the combined XY hull of scan + floor grid
    """
    pts = cloud.point["positions"].numpy()

    if z_floor is None:
        z_floor = float(pts[:, 2].min())

    if bottom_spacing is None:
        # Match approximate cloud density
        #from scipy.spatial import KDTree
        dists, _ = KDTree(pts).query(pts, k=2, workers=-1)
        bottom_spacing = float(np.median(dists[:, 1]))

    # --- Generate flat grid of synthetic points at z_floor ---
    x_min, x_max = pts[:, 0].min(), pts[:, 0].max()
    y_min, y_max = pts[:, 1].min(), pts[:, 1].max()

    x_grid = np.arange(x_min, x_max + bottom_spacing, bottom_spacing)
    y_grid = np.arange(y_min, y_max + bottom_spacing, bottom_spacing)
    XX, YY = np.meshgrid(x_grid, y_grid)
    grid_pts = np.column_stack([XX.ravel(), YY.ravel(),
                                np.full(XX.size, z_floor)])

    # Restrict bottom grid to XY convex hull of original cloud
    # hull_2d   = Delaunay(pts[:, :2])
    # inside    = hull_2d.find_simplex(grid_pts[:, :2]) >= 0
    # bottom_pts = grid_pts[inside]

    # print(f"  Added {len(bottom_pts):,} synthetic bottom points at z={z_floor:.4f}")
    # --- Hull from bottom-most points only ---
    z_range           = pts[:, 2].max() - pts[:, 2].min()
    z_bottom_thresh   = z_floor + z_range * 0.1        # bottom 10% of object height
    bottom_cloud_pts  = pts[pts[:, 2] <= z_bottom_thresh]

    if len(bottom_cloud_pts) < 4:
        hull_2d = Delaunay(pts[:, :2])                 # fallback: full cloud hull
    else:
        hull_2d = Delaunay(bottom_cloud_pts[:, :2])    # hull from bottom ring only

    # Filter grid to hull footprint
    inside            = hull_2d.find_simplex(grid_pts[:, :2]) >= 0
    candidates        = grid_pts[inside]

    # --- Remove grid points where original cloud already has XY coverage ---
    tree_2d = KDTree(pts[:, :2])
    dists, _ = tree_2d.query(candidates[:, :2], k=1, workers=-1)
    no_coverage= dists > bottom_spacing * 0.5   # no original point within half a cell
    #bottom_pts= candidates[no_coverage]
    bottom_pts = candidates

    print(f"  Added {len(bottom_pts):,} synthetic bottom points at z={z_floor:.4f}")
    # --- Combine original + synthetic points ---
    all_pts = np.vstack([pts, bottom_pts])
    legacy  = o3d.geometry.PointCloud()
    legacy.points = o3d.utility.Vector3dVector(all_pts)

    # Estimate normals on combined cloud
    legacy.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
    legacy.orient_normals_consistent_tangent_plane(k=15)

    # Force synthetic bottom points to have downward normals (0, 0, -1)
    # This is critical — it tells Poisson there is a flat downward surface here
    normals = np.asarray(legacy.normals)
    # normals[len(pts):] = [0.0, 0.0, -1.0]
    legacy.normals = o3d.utility.Vector3dVector(normals)
    pv_cloud = pv.PolyData(all_pts)

# Colour by source: 0 = original scan, 1 = synthetic bottom
    source_labels = np.zeros(len(all_pts))
    source_labels[len(pts):] = 1
    pv_cloud["source"] = source_labels

    plotter = pv.Plotter()
    plotter.add_points(pv_cloud, scalars="source",
                    cmap=["steelblue", "red"],
                    point_size=2,
                    render_points_as_spheres=True)
    plotter.add_scalar_bar(title="Blue = scan  |  Red = synthetic floor")
    plotter.show(cpos='xy')
    # --- Poisson reconstruction ---
    #with suppress_cpp_output():
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(legacy, depth=depth)

    # Density pruning
    if density_quantile > 0:
        densities = np.asarray(densities)
        remove    = densities < np.quantile(densities, density_quantile)
        mesh.remove_vertices_by_mask(remove)

    # Crop to XY convex hull of original cloud (not bottom grid)
    # verts   = np.asarray(mesh.vertices)
    # outside = hull_2d.find_simplex(verts[:, :2]) < 0
    # mesh.remove_vertices_by_mask(outside)

    # Crop to XY hull of original cloud + bottom points combined
    combined_xy = np.vstack([pts[:, :2], bottom_pts[:, :2]])
    hull_combined = Delaunay(combined_xy)

    verts   = np.asarray(mesh.vertices)
    outside = hull_combined.find_simplex(verts[:, :2]) < 0
    mesh.remove_vertices_by_mask(outside)
    mesh.compute_vertex_normals()
    return mesh

def add_bottom(cloud, depth=9, density_quantile=0.05,
                                    z_floor=None, bottom_spacing=None):
    """
    Poisson reconstruction with a synthetic plane of points added at z_floor
    before reconstruction. Poisson naturally incorporates a flat bottom
    without any post-processing mesh surgery.

    Parameters
    ----------
    cloud           : o3d.t.geometry.PointCloud
    depth           : int   — Poisson octree depth
    density_quantile: float — density pruning threshold
    z_floor         : float — Z height of flat bottom (default: cloud z_min)
    bottom_spacing  : float — grid spacing for synthetic bottom points
                              (default: matches cloud average point spacing)

    Returns
    -------
    pv.PolyData — combined point cloud (scan + synthetic floor) with a "source"
                  scalar field (0 = original scan, 1 = synthetic floor), for
                  inspection before reconstruction
    """
    pts = cloud.point["positions"].numpy()

    if z_floor is None:
        z_floor = float(pts[:, 2].min())

    if bottom_spacing is None:
        # Match approximate cloud density
        #from scipy.spatial import KDTree
        dists, _ = KDTree(pts).query(pts, k=2, workers=-1)
        bottom_spacing = float(np.median(dists[:, 1]))

    # --- Generate flat grid of synthetic points at z_floor ---
    x_min, x_max = pts[:, 0].min(), pts[:, 0].max()
    y_min, y_max = pts[:, 1].min(), pts[:, 1].max()

    x_grid = np.arange(x_min, x_max + bottom_spacing, bottom_spacing)
    y_grid = np.arange(y_min, y_max + bottom_spacing, bottom_spacing)
    XX, YY = np.meshgrid(x_grid, y_grid)
    grid_pts = np.column_stack([XX.ravel(), YY.ravel(),
                                np.full(XX.size, z_floor)])

    # Restrict bottom grid to XY convex hull of original cloud
    # hull_2d   = Delaunay(pts[:, :2])
    # inside    = hull_2d.find_simplex(grid_pts[:, :2]) >= 0
    # bottom_pts = grid_pts[inside]

    # print(f"  Added {len(bottom_pts):,} synthetic bottom points at z={z_floor:.4f}")
    # --- Hull from bottom-most points only ---
    z_range           = pts[:, 2].max() - pts[:, 2].min()
    z_bottom_thresh   = z_floor + z_range * 0.1        # bottom 10% of object height
    bottom_cloud_pts  = pts[pts[:, 2] <= z_bottom_thresh]

    if len(bottom_cloud_pts) < 4:
        hull_2d = Delaunay(pts[:, :2])                 # fallback: full cloud hull
    else:
        hull_2d = Delaunay(bottom_cloud_pts[:, :2])    # hull from bottom ring only

    # Filter grid to hull footprint
    inside            = hull_2d.find_simplex(grid_pts[:, :2]) >= 0
    candidates        = grid_pts[inside]

    # --- Remove grid points where original cloud already has XY coverage ---
    tree_2d = KDTree(pts[:, :2])
    dists, _ = tree_2d.query(candidates[:, :2], k=1, workers=-1)
    no_coverage= dists > bottom_spacing * 0.5   # no original point within half a cell
    #bottom_pts= candidates[no_coverage]
    bottom_pts = candidates

    print(f"  Added {len(bottom_pts):,} synthetic bottom points at z={z_floor:.4f}")
    # --- Combine original + synthetic points ---
    all_pts = np.vstack([pts, bottom_pts])
    legacy  = o3d.geometry.PointCloud()
    legacy.points = o3d.utility.Vector3dVector(all_pts)

    # Estimate normals on combined cloud
    legacy.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
    legacy.orient_normals_consistent_tangent_plane(k=15)

    # Force synthetic bottom points to have downward normals (0, 0, -1)
    # This is critical — it tells Poisson there is a flat downward surface here
    normals = np.asarray(legacy.normals)
    # normals[len(pts):] = [0.0, 0.0, -1.0]
    legacy.normals = o3d.utility.Vector3dVector(normals)
    pv_cloud = pv.PolyData(all_pts)

# Colour by source: 0 = original scan, 1 = synthetic bottom
    source_labels = np.zeros(len(all_pts))
    source_labels[len(pts):] = 1
    pv_cloud["source"] = source_labels

    return pv_cloud