import open3d as o3d
import numpy as np
import sys


def create_letter_points(letter, offset=(0, 0, 0), scale=0.15):

    pts = []

    if letter == "X":
        t = np.linspace(0, 1, 50)
        pts1 = np.column_stack((t, t, np.zeros_like(t)))
        pts2 = np.column_stack((t, 1 - t, np.zeros_like(t)))
        pts = np.vstack((pts1, pts2))

    elif letter == "Y":
        t = np.linspace(0, 1, 30)

        arm1 = np.column_stack((0.5 * t, 1 - 0.5 * t, np.zeros_like(t)))
        arm2 = np.column_stack((0.5 + 0.5 * t, 0.5 + 0.5 * t, np.zeros_like(t)))
        stem = np.column_stack((
            0.5 * np.ones_like(t),
            0.5 * (1 - t),
            np.zeros_like(t)
        ))

        pts = np.vstack((arm1, arm2, stem))

    elif letter == "Z":
        t = np.linspace(0, 1, 50)

        top = np.column_stack((t, np.ones_like(t), np.zeros_like(t)))
        diag = np.column_stack((1 - t, 1 - t, np.zeros_like(t)))
        bottom = np.column_stack((t, np.zeros_like(t), np.zeros_like(t)))

        pts = np.vstack((top, diag, bottom))

    pts *= scale
    pts += np.array(offset)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)

    # Black text
    pcd.paint_uniform_color([0, 0, 0])

    return pcd


def create_grid(size=10, spacing=1.0):

    points = []
    lines = []
    colors = []

    idx = 0

    vals = np.arange(-size, size + spacing, spacing)

    for v in vals:

        # Lines parallel to X
        points.append([-size, v, 0])
        points.append([ size, v, 0])

        lines.append([idx, idx + 1])
        colors.append([0.7, 0.7, 0.7])  # light grey

        idx += 2

        # Lines parallel to Y
        points.append([v, -size, 0])
        points.append([v,  size, 0])

        lines.append([idx, idx + 1])
        colors.append([0.7, 0.7, 0.7])

        idx += 2

    grid = o3d.geometry.LineSet()

    grid.points = o3d.utility.Vector3dVector(points)
    grid.lines = o3d.utility.Vector2iVector(lines)
    grid.colors = o3d.utility.Vector3dVector(colors)

    return grid


def main():

    if len(sys.argv) < 2:
        print("Usage: python view_ply_grid.py cloud.ply")
        return

    ply_path = sys.argv[1]

    # Load cloud
    pcd = o3d.io.read_point_cloud(ply_path)

    if pcd.is_empty():
        print("Failed to load point cloud.")
        return

    # Cloud scale
    bbox = pcd.get_axis_aligned_bounding_box()
    extent = bbox.get_extent()
    scale = np.max(extent)

    axis_length = scale * 0.25
    text_scale = scale * 0.03

    # ----- AXES -----

    axis_points = [
        [0, 0, 0],
        [axis_length, 0, 0],
        [0, axis_length, 0],
        [0, 0, axis_length]
    ]

    axis_lines = [
        [0, 1],
        [0, 2],
        [0, 3]
    ]

    axis_colors = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ]

    axes = o3d.geometry.LineSet()
    axes.points = o3d.utility.Vector3dVector(axis_points)
    axes.lines = o3d.utility.Vector2iVector(axis_lines)
    axes.colors = o3d.utility.Vector3dVector(axis_colors)

    # ----- LABELS -----

    x_label = create_letter_points(
        "X",
        offset=(axis_length * 1.05, 0, 0),
        scale=text_scale
    )

    y_label = create_letter_points(
        "Y",
        offset=(0, axis_length * 1.05, 0),
        scale=text_scale
    )

    z_label = create_letter_points(
        "Z",
        offset=(0, 0, axis_length * 1.05),
        scale=text_scale
    )

    # ----- GRID -----

    grid_size = max(5, int(scale))
    grid_spacing = max(scale / 20, 0.5)

    grid = create_grid(
        size=grid_size,
        spacing=grid_spacing
    )

    # ----- VISUALIZE -----

    o3d.visualization.draw_geometries(
        [pcd, grid, axes, x_label, y_label, z_label],
        window_name="PLY Viewer with Grid",
        width=1400,
        height=900
    )


if __name__ == "__main__":
    main()