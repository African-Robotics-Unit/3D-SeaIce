import numpy as np
import open3d as o3d

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


def apply_rs_tforms(clouds, tforms):
    """Apply rigid transforms with ROS-to-local coordinate conversion."""
    C_RL = np.array([
    [0,  0, -1, 0],
    [-1, 0,  0, 0],
    [0,  1,  0, 0],
    [0,  0,  0, 1],
    ], dtype=float)

    C_LR = C_RL.T
    transformed = []
    for i in range(len(clouds)):
        total_tform = C_RL @ tforms[i] @ C_LR
        cloud_t = clouds[i].clone()
        cloud_t.transform(total_tform)
        transformed.append(cloud_t)
    return transformed


def save_clouds(clouds, export_path, fname):
    """Write each cloud to a PLY file named {fname}{A-H}.ply."""
    from pathlib import Path
    export_path = Path(export_path)
    export_path.mkdir(parents=True, exist_ok=True)
    letters = "ABCDEFGH"
    for k, cloud in enumerate(clouds):
        filepath = export_path / f"{fname}{letters[k]}.ply"
        o3d.t.io.write_point_cloud(str(filepath), cloud)