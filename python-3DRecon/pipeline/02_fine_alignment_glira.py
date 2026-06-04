#!/usr/bin/env python3
"""
fine_alignment_glira.py
-----------------------
Python port of GliraICP (globalICP.runICP) — Glira et al. 2015.
No MATLAB engine required.

Per-iteration pipeline (mirrors runICP.m exactly):
  1. Voxel hulls  →  pairwise overlap detection
  2. Selection    →  uniform voxel sample of query points restricted to overlap
  3. Matching     →  KNN(1) + SVD plane normals (from full cloud) + dp / ds / dAlpha
  4. Rejection    →  dAlpha, dp-MAD, ds, roughness thresholds
  5. Weighting    →  roughness and normal-angle weights
  6. Minimization →  global weighted point-to-plane least squares (all pairs at once)
  7. Transform    →  apply dH to working clouds, accumulate H_total
  Final: apply H_total to original clouds and save

Requirements: numpy  scipy  open3d

Auto-parameter formulas (from runICPInit.m):
  HullVoxelSize = 5   × PlaneSearchRadius
  MaxDistance   = 3   × PlaneSearchRadius
  MaxRoughness  = 0.2 × PlaneSearchRadius
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import argparse
import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple
import tools.pcio as pcio
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree

log = logging.getLogger(__name__)


# ── Parameters ─────────────────────────────────────────────────────────────────

@dataclass
class ICPParams:
    max_no_it: int = 5
    idx_fixed: Set[int] = field(default_factory=lambda: {0})  # 0-indexed; PC 0 is fixed

    no_of_trafo_param: int = 6          # rigid body (3 rot + 3 trans)

    hull_voxel_size: float = 0.0        # 0 = auto (5 × plane_search_radius)

    uniform_sampling_distance: float = 0.2
    plane_search_radius: float = 0.1

    max_roughness: float = 0.0          # 0 = auto (0.2 × plane_search_radius)
    max_delta_angle: float = 10.0       # degrees
    max_distance: float = 0.0           # 0 = auto (3 × plane_search_radius)
    max_sigma_mad: float = 3.0

    weight_by_roughness: bool = True
    weight_by_delta_angle: bool = True

    stop_condition_normdx: float = -1.0  # negative = never stop early


# ── Correspondence container ───────────────────────────────────────────────────

@dataclass
class Corr:
    X1: np.ndarray        # (N,3) query points  (PC i)
    X2: np.ndarray        # (N,3) matched points (PC j)
    n1: np.ndarray        # (N,3) normals at X1
    n2: np.ndarray        # (N,3) normals at X2
    dp: np.ndarray        # (N,)  n1·(X1−X2)  [point-to-plane distance]
    ds: np.ndarray        # (N,)  ‖X1−X2‖     [point-to-point distance]
    d_alpha: np.ndarray   # (N,)  angle(n1,n2) [degrees]
    r1: np.ndarray        # (N,)  roughness at X1
    r2: np.ndarray        # (N,)  roughness at X2
    w: np.ndarray         # (N,)  weights [0,1]

    def filter(self, keep: np.ndarray) -> "Corr":
        return Corr(**{k: v[keep] for k, v in self.__dict__.items()})


# ── I/O ────────────────────────────────────────────────────────────────────────

def load_point_clouds(folder: str) -> Tuple[List[Path], List[np.ndarray]]:
    paths = sorted(Path(folder).glob("*.ply"))
    if not paths:
        raise FileNotFoundError(f"No .ply files in {folder}")
    pts_list = []
    for p in paths:
        pc = o3d.io.read_point_cloud(str(p))
        if pc.is_empty():
            raise ValueError(f"Empty cloud: {p}")
        pts_list.append(np.asarray(pc.points, dtype=np.float64))
    log.info(f"Loaded {len(pts_list)} point clouds")
    return paths, pts_list


def save_point_clouds(paths: List[Path], pts_list: List[np.ndarray], out_folder: str):
    out = Path(out_folder)
    out.mkdir(parents=True, exist_ok=True)
    for p, pts in zip(paths, pts_list):
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(pts)
        o3d.io.write_point_cloud(str(out / p.name), pc)


# ── Voxel utilities ────────────────────────────────────────────────────────────

def voxel_hull(pts: np.ndarray, voxel_size: float) -> Set[Tuple[int, int, int]]:
    """Set of occupied voxel cells at the given resolution."""
    idx = np.floor(pts / voxel_size).astype(np.int64)
    return set(map(tuple, idx))


def build_pairs(hulls: List[Set]) -> List[Tuple[int, int]]:
    """Return (i,j) pairs whose voxel hulls share at least one voxel."""
    n = len(hulls)
    return [(i, j) for i in range(n) for j in range(i + 1, n) if hulls[i] & hulls[j]]


def uniform_sample_in_overlap(
    pts: np.ndarray,
    search_hull: Set[Tuple[int, int, int]],
    hull_size: float,
    sample_size: float,
) -> np.ndarray:
    """
    1. Keep only points whose hull-voxel is in search_hull (overlap region).
    2. Uniformly downsample at sample_size: one point per cell, closest to center.
    Returns indices into pts.
    """
    h_idx = np.floor(pts / hull_size).astype(np.int64)
    in_overlap = np.array([tuple(r) in search_hull for r in h_idx], dtype=bool)
    overlap_idx = np.where(in_overlap)[0]
    if len(overlap_idx) == 0:
        return np.empty(0, dtype=np.int64)

    sub = pts[overlap_idx]
    cell = np.floor(sub / sample_size).astype(np.int64)
    centers = (cell + 0.5) * sample_size
    dist_to_center = np.linalg.norm(sub - centers, axis=1)

    best: Dict[Tuple, int] = {}
    for k, c in enumerate(map(tuple, cell)):
        if c not in best or dist_to_center[k] < dist_to_center[best[c]]:
            best[c] = k

    return overlap_idx[np.array(list(best.values()), dtype=np.int64)]


# ── Normal + roughness estimation ──────────────────────────────────────────────

def _fit_plane(nb_pts: np.ndarray) -> Tuple[Optional[np.ndarray], float]:
    """SVD plane fit. Requires >= 8 points (matches GliraICP minimum)."""
    if len(nb_pts) < 8:
        return None, np.nan
    c = nb_pts.mean(axis=0)
    _, _, Vt = np.linalg.svd(nb_pts - c, full_matrices=False)
    normal = Vt[-1]
    roughness = float(np.std((nb_pts - c) @ normal))
    return normal, roughness


def compute_normals_roughness(
    cloud_pts: np.ndarray,   # full cloud to search neighbours in
    query_pts: np.ndarray,   # points at which to estimate normals
    radius: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    For each point in query_pts, fit a plane to neighbours in cloud_pts
    within radius. Returns normals (M,3) and roughness (M,), NaN where
    the plane fit fails (< 8 neighbours).
    """
    tree = KDTree(cloud_pts)
    m = len(query_pts)
    normals = np.full((m, 3), np.nan)
    roughness = np.full(m, np.nan)
    for k, nb_idx in enumerate(tree.query_ball_point(query_pts, radius, workers=-1)):
        n, r = _fit_plane(cloud_pts[nb_idx])
        if n is not None:
            normals[k] = n
            roughness[k] = r
    return normals, roughness


# ── Matching ───────────────────────────────────────────────────────────────────

def match_and_compute(
    query_pts: np.ndarray,          # selected (downsampled) points to match FROM
    search_pts: np.ndarray,         # full cloud to match INTO (PC j)
    plane_radius: float,
    full_query_cloud: np.ndarray,   # FIX 1: full PC i used for normal estimation
) -> Corr:
    """
    KNN(1) query→search, then estimate normals + roughness using the FULL
    source/target clouds (not the downsampled subset). This matches GliraICP's
    behaviour where pointCloud.normals() operates on all cloud points.
    """
    _, match_idx = KDTree(search_pts).query(query_pts, k=1, workers=-1)
    X1 = query_pts
    X2 = search_pts[match_idx]

    # FIX 1: use full dense clouds for plane fitting, not the ~300-pt subset.
    # With plane_radius=0.1 m and ~300 pts at 0.2 m spacing, the subset gives
    # < 8 neighbours → all NaN normals → all correspondences rejected.
    n1, r1 = compute_normals_roughness(full_query_cloud, X1, plane_radius)
    n2, r2 = compute_normals_roughness(search_pts,       X2, plane_radius)

    # Orient n2 consistent with n1 (flip where dot product < 0)
    flip = np.nansum(n1 * n2, axis=1) < 0
    n2[flip] = -n2[flip]

    dp      = np.sum(n1 * (X1 - X2), axis=1)           # NaN propagates from n1
    ds      = np.linalg.norm(X1 - X2, axis=1)
    dot     = np.clip(np.sum(n1 * n2, axis=1), -1.0, 1.0)
    d_alpha = np.degrees(np.arccos(dot))                # NaN where either normal is NaN

    return Corr(X1=X1, X2=X2, n1=n1, n2=n2,
                dp=dp, ds=ds, d_alpha=d_alpha,
                r1=r1, r2=r2, w=np.ones(len(X1)))


# ── Rejection ──────────────────────────────────────────────────────────────────

def rejection(corr: Corr, p: ICPParams) -> Corr:
    """Four GliraICP rejection criteria, applied in order (mirrors runICPRejection.m)."""
    keep = np.ones(len(corr.X1), dtype=bool)

    # 1. Failed normal estimation (NaN normal → NaN dp/dAlpha → caught here)
    keep &= ~np.any(np.isnan(corr.n1), axis=1)
    keep &= ~np.any(np.isnan(corr.n2), axis=1)

    # 2. Normal angle > MaxDeltaAngle (degrees)
    keep &= corr.d_alpha <= p.max_delta_angle

    # 3. Robust MAD outlier rejection on dp (1.4826 × MAD, computed after dAlpha filter)
    dp_kept = corr.dp[keep]
    if dp_kept.size > 0:
        med     = float(np.median(dp_kept))
        sig_mad = 1.4826 * float(np.median(np.abs(dp_kept - med)))
        keep   &= (corr.dp >= med - p.max_sigma_mad * sig_mad) & \
                  (corr.dp <= med + p.max_sigma_mad * sig_mad)

    # 4. Point-to-point distance
    keep &= corr.ds <= p.max_distance

    # 5. Roughness
    keep &= np.maximum(corr.r1, corr.r2) <= p.max_roughness

    return corr.filter(keep)


# ── Weighting ──────────────────────────────────────────────────────────────────

def weighting(corr: Corr, p: ICPParams) -> Corr:
    """Scale correspondence weights by roughness and/or normal angle."""
    w = corr.w.copy()

    if p.weight_by_roughness:
        r_max      = np.maximum(corr.r1, corr.r2)
        global_max = float(r_max.max()) if r_max.size > 0 else 1.0
        if global_max > 0:
            w *= 1.0 - r_max / global_max

    if p.weight_by_delta_angle:
        w *= np.abs(np.cos(np.radians(corr.d_alpha)))

    return Corr(**{**corr.__dict__, "w": np.clip(w, 0.0, 1.0)})


# ── Rotation helpers ───────────────────────────────────────────────────────────

def opk_to_R(omega: float, phi: float, kappa: float) -> np.ndarray:
    """
    Rotation matrix from omega (x-axis), phi (y-axis), kappa (z-axis) in radians.
    R = Rx(omega) · Ry(phi) · Rz(kappa) — matches GliraICP's opk2R convention.
    """
    co, so = np.cos(omega), np.sin(omega)
    cp, sp = np.cos(phi),   np.sin(phi)
    ck, sk = np.cos(kappa), np.sin(kappa)
    Rx = np.array([[1,  0,   0 ], [0,  co, -so], [0,  so,  co]])
    Ry = np.array([[cp, 0,  sp ], [0,   1,   0 ], [-sp, 0,  cp]])
    Rz = np.array([[ck, -sk, 0 ], [sk, ck,   0 ], [0,   0,   1]])
    return Rx @ Ry @ Rz


def make_H(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    H = np.eye(4)
    H[:3, :3] = R
    H[:3,  3] = t
    return H


# ── Global weighted point-to-plane least squares ───────────────────────────────

def global_minimization(
    pair_corrs: List[Tuple[Tuple[int, int], Corr]],
    n_pcs: int,
    fixed_set: Set[int],
) -> Tuple[List[np.ndarray], float]:
    """
    Solve for all loose-PC differential rigid transforms simultaneously
    (mirrors lsAdj.solve called inside runICPMinimization.m).

    Linearized point-to-plane condition for pair (i,j), correspondence k:
        (x1×n1)·δω_i + n1·δt_i  −  (x2×n1)·δω_j − n1·δt_j  =  −dp

    Parameters per loose PC (column order): [δωx, δωy, δωz, δtx, δty, δtz]

    Returns (list of 4×4 dH per PC, ‖δx‖).
    """
    loose     = [i for i in range(n_pcs) if i not in fixed_set]
    col_start = {pc: 6 * k for k, pc in enumerate(loose)}
    n_params  = 6 * len(loose)

    A_blocks, b_blocks, w_blocks = [], [], []

    for (i, j), corr in pair_corrs:
        if len(corr.X1) == 0:
            continue
        X1, X2, N1, w = corr.X1, corr.X2, corr.n1, corr.w
        n_c   = len(X1)
        A_p   = np.zeros((n_c, n_params))

        if i not in fixed_set:
            c = col_start[i]
            A_p[:, c:c+3]   =  np.cross(X1, N1)   #  (x1 × n1) for δω_i
            A_p[:, c+3:c+6] =  N1                  #  n1        for δt_i

        if j not in fixed_set:
            c = col_start[j]
            A_p[:, c:c+3]   = -np.cross(X2, N1)   # -(x2 × n1) for δω_j
            A_p[:, c+3:c+6] = -N1                  # -n1        for δt_j

        b_p = -np.einsum("ij,ij->i", N1, X1 - X2)  # −dp (residual to close)

        A_blocks.append(A_p)
        b_blocks.append(b_p)
        w_blocks.append(w)

    if not A_blocks:
        return [np.eye(4)] * n_pcs, 0.0

    A      = np.vstack(A_blocks)
    b      = np.concatenate(b_blocks)
    w_sqrt = np.sqrt(np.clip(np.concatenate(w_blocks), 0, None))

    # Weighted least squares: scale rows by sqrt(w) → standard lstsq
    x, _, _, _ = np.linalg.lstsq(A * w_sqrt[:, None], b * w_sqrt, rcond=None)
    norm_dx    = float(np.linalg.norm(x))

    dH = []
    for i in range(n_pcs):
        if i in fixed_set:
            dH.append(np.eye(4))
        else:
            c = col_start[i]
            R = opk_to_R(x[c], x[c + 1], x[c + 2])
            t = x[c + 3:c + 6]
            dH.append(make_H(R, t))

    return dH, norm_dx


# ── Main ICP loop ──────────────────────────────────────────────────────────────

def run_icp(pts_orig: List[np.ndarray], p: ICPParams) -> List[np.ndarray]:
    """
    Run GliraICP algorithm on a list of (N,3) point cloud arrays.
    Returns list of final 4×4 homogeneous transforms (identity for fixed clouds).
    """
    n_pcs = len(pts_orig)
    pts   = [pc.copy() for pc in pts_orig]   # working copies, updated each iteration
    H_total = [np.eye(4) for _ in range(n_pcs)]

    # FIX 3: auto-parameter formulas from runICPInit.m
    # HullVoxelSize = 5   × PlaneSearchRadius  (log: 5  × 0.1 = 0.50)
    # MaxDistance   = 3   × PlaneSearchRadius  (log: 3  × 0.1 = 0.30)
    # MaxRoughness  = 0.2 × PlaneSearchRadius  (log: 0.2× 0.1 = 0.02)
    if p.hull_voxel_size <= 0:
        p.hull_voxel_size = 5.0 * p.plane_search_radius
    if p.max_distance <= 0:
        p.max_distance = 3.0 * p.plane_search_radius
    if p.max_roughness <= 0:
        p.max_roughness = 0.2 * p.plane_search_radius

    log.info(f"Effective params: hull_voxel={p.hull_voxel_size:.3f}  "
             f"max_dist={p.max_distance:.3f}  max_rough={p.max_roughness:.4f}")

    # Initial pair list from voxel hull overlap
    hulls     = [voxel_hull(pc, p.hull_voxel_size) for pc in pts]
    pair_list = build_pairs(hulls)

    if not pair_list:
        log.warning("No overlapping pairs found — check hull_voxel_size.")
        return H_total

    log.info(f"Pair list: {pair_list}")

    for it in range(1, p.max_no_it + 1):
        log.info(f"\n=== ICP ITERATION {it}/{p.max_no_it} ===")

        if it > 1:
            hulls = [voxel_hull(pc, p.hull_voxel_size) for pc in pts]

        pair_corrs = []

        for (i, j) in pair_list:
            # Selection: uniform sample of query pts (PC i) inside overlap with PC j
            q_idx = uniform_sample_in_overlap(
                pts[i], hulls[j], p.hull_voxel_size, p.uniform_sampling_distance)

            if len(q_idx) < p.no_of_trafo_param:
                log.debug(f"  ({i},{j}): {len(q_idx)} query pts — skip")
                continue

            # FIX 2: pass pts[i] (full cloud) as full_query_cloud
            corr = match_and_compute(
                pts[i][q_idx],          # downsampled query points (correspondence seeds)
                pts[j],                 # full search cloud
                p.plane_search_radius,
                pts[i],                 # FIX 2: full query cloud for normal estimation
            )

            # Rejection
            n_before = len(corr.X1)
            corr     = rejection(corr, p)
            n_after  = len(corr.X1)
            log.info(f"  Pair ({i},{j}): {n_before} → {n_after} correspondences")

            if n_after < p.no_of_trafo_param:
                log.warning(f"  Pair ({i},{j}): too few after rejection — skip")
                continue

            # Weighting
            corr = weighting(corr, p)
            pair_corrs.append(((i, j), corr))

        if not pair_corrs:
            log.warning("No valid pairs this iteration — stopping.")
            break

        # Global minimization: all pairs solved simultaneously
        dH_list, norm_dx = global_minimization(pair_corrs, n_pcs, p.idx_fixed)
        log.info(f"  ‖δx‖ = {norm_dx:.6e}")

        # Apply differential transforms and accumulate
        for i in range(n_pcs):
            if i not in p.idx_fixed:
                R = dH_list[i][:3, :3]
                t = dH_list[i][:3, 3]
                pts[i]     = (R @ pts[i].T).T + t
                H_total[i] = dH_list[i] @ H_total[i]

        if p.stop_condition_normdx > 0 and norm_dx <= p.stop_condition_normdx:
            log.info("  Convergence criterion met — stopping.")
            break

    return H_total


# ── CLI ────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="GliraICP-equivalent fine alignment (Python, no MATLAB engine)")
    ap.add_argument("folder",            help="Folder containing .ply input files")
    ap.add_argument("--out",             default="aligned",
                    help="Output folder for aligned .ply files  [default: aligned/]")
    ap.add_argument("--config",          default=None,
                    help="JSON config with 'ICPOptions' key (same schema as MATLAB)")
    ap.add_argument("--uniform",         type=float, default=0.2,
                    help="UniformSamplingDistance (m)  [default: 0.2]")
    ap.add_argument("--plane-radius",    type=float, default=0.1,
                    help="PlaneSearchRadius (m)         [default: 0.1]")
    ap.add_argument("--max-roughness",   type=float, default=0.0,
                    help="MaxRoughness (0=auto: 0.2×PlaneSearchRadius) [default: auto]")
    ap.add_argument("--max-iter",        type=int,   default=5)
    ap.add_argument("--save-transforms", default="transforms.json",
                    help="Path to output JSON with 4×4 transform matrices")
    ap.add_argument("--visualize",       action="store_true")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(message)s")

    p = ICPParams()

    if args.config:
        with open(args.config) as f:
            cfg = json.load(f)
        o = cfg.get("ICPOptions", {})
        if "NoOfTransfParam"         in o: p.no_of_trafo_param         = o["NoOfTransfParam"]
        if "UniformSamplingDistance" in o: p.uniform_sampling_distance  = o["UniformSamplingDistance"]
        if "PlaneSearchRadius"       in o: p.plane_search_radius         = o["PlaneSearchRadius"]
        if "MaxRoughness"            in o: p.max_roughness               = o["MaxRoughness"]
        if "MaxNoIt"                 in o: p.max_no_it                   = o["MaxNoIt"]
        if "MaxDeltaAngle"           in o: p.max_delta_angle             = o["MaxDeltaAngle"]
        if "MaxSigmaMad"             in o: p.max_sigma_mad               = o["MaxSigmaMad"]
    else:
        p.uniform_sampling_distance = args.uniform
        p.plane_search_radius       = args.plane_radius
        p.max_roughness             = args.max_roughness
        p.max_no_it                 = args.max_iter

    paths, pts_list = load_point_clouds(args.folder)

    H_list = run_icp(pts_list, p)

    # Save transform matrices

    # with open(args.save_transforms, "w") as f:
    #     json.dump({str(paths[i].name): H_list[i].tolist()
    #                for i in range(len(paths))}, f, indent=2)
    # print(f"Transforms → {args.save_transforms}")

    CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'p3_config.json')
    with open(CONFIG_PATH, "r") as f:
        config = json.load(f)
    scan_labels = [p.name for p in paths]
    after_icp_path = config["outputFolder"] + config["preprocessing"]["afterICPpath"]
    aln_path = after_icp_path + "ICPtforms.aln"
    pcio.write_aln(aln_path, H_list, scan_labels)

    # Apply H_total to original clouds and save
    pts_aligned = []
    for i, pts in enumerate(pts_list):
        R = H_list[i][:3, :3]
        t = H_list[i][:3, 3]
        pts_aligned.append((R @ pts.T).T + t)

    save_point_clouds(paths, pts_aligned, args.out)
    print(f"Aligned clouds → {args.out}/")

    if args.visualize:
        palette = [[1,0,0],[0,1,0],[0,0,1],[1,1,0],[0,1,1],[1,0,1]]
        vis = []
        for i, pts in enumerate(pts_aligned):
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(pts)
            pc.paint_uniform_color(palette[i % len(palette)])
            vis.append(pc)
        o3d.visualization.draw_geometries(vis)


if __name__ == "__main__":
    main()
