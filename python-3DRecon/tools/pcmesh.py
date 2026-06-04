import numpy as np
import open3d as o3d
import contextlib
import os
import pymeshfix
import trimesh
from shapely.geometry import Polygon
import pyvista as pv
from scipy.spatial import KDTree, Delaunay


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
