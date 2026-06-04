import numpy as np
import open3d as o3d
from pathlib import Path


# --- .aln transform file I/O (pcalign.py versions — used by pipeline) ---

def read_aln(filepath):
    with open(filepath, "r") as f:
        n_scans = int(f.readline().strip())
        transforms = []
        labels = []
        for i in range(n_scans):
            label = f.readline().strip()
            f.readline()  # skip '#' line
            rows = [list(map(float, f.readline().split())) for row in range(4)]
            transforms.append(np.array(rows))
            labels.append(label)
    transforms = np.array(transforms)
    return transforms, labels


def write_aln(filepath, transforms, labels):
    with open(filepath, "w") as f:
        f.write(f"{len(transforms)}\n")
        for label, tform in zip(labels, transforms):
            f.write(f"{label}\n")
            f.write("#\n")
            for row in tform:
                f.write(" ".join(f"{v:.6f}" for v in row) + " \n")
        f.write("0\n")


def save_clouds(clouds, export_path, fname):
    """Write each cloud to a PLY file named {fname}{A-H}.ply."""
    export_path = Path(export_path)
    export_path.mkdir(parents=True, exist_ok=True)
    letters = "ABCDEFGH"
    for k, cloud in enumerate(clouds):
        filepath = export_path / f"{fname}{letters[k]}.ply"
        o3d.t.io.write_point_cloud(str(filepath), cloud)


def read_clouds(input_path):
    """
    Read all .ply files in input_path (sorted alphabetically).
    Returns list of o3d.t.geometry.PointCloud.
    Intensity and all other attributes are accessible via cloud.point["attribute_name"].
    """
    input_path = Path(input_path)
    paths = sorted(input_path.glob("*.ply"))

    if not paths:
        raise FileNotFoundError(f"No .ply files found in {input_path}")

    clouds = []
    for filepath in paths:
        cloud = o3d.t.io.read_point_cloud(str(filepath))
        if cloud.is_empty():
            raise ValueError(f"Empty point cloud loaded from {filepath}")
        clouds.append(cloud)

        n_pts = len(cloud.point["positions"])

        # TensorMap does not support .keys() — check each attribute individually
        found = []
        for attr in ["intensity", "colors", "normals"]:
            try:
                cloud.point[attr]
                found.append(attr)
            except KeyError:
                pass

        has_intensity = "intensity" in found
        print(f"  {filepath.name}: {n_pts:,} points | "
              f"attributes: {found} | "
              f"intensity: {'YES' if has_intensity else 'NOT FOUND'}")

    return clouds


# --- Single cloud I/O and format conversion (pctools.py) ---

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
