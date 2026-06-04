import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import json
import ast
from pathlib import Path
from tools.pcdisplay import show_pointcloud_intensity, plotter_pcdisplay
import numpy as np
import pyvista as pv
import re
from tools.roiutils import decode_bound, encode_roi

def parse_roiNEW(user_input):
    # Convert unquoted inf values to quoted strings before ast.literal_eval
    cleaned = re.sub(r'(?<![\w"\'])-inf(?![\w"\'])', '"-inf"', user_input)
    cleaned = re.sub(r'(?<![\w"\'])inf(?![\w"\'])', '"inf"', cleaned)

    roi = ast.literal_eval(cleaned)

    if len(roi) != 3:
        raise ValueError("ROI must contain x, y and z bounds.")

    def convert(value):
        if isinstance(value, str):
            value = value.lower().strip()
            if value == "-inf":
                return -np.inf
            if value == "inf":
                return np.inf
        return float(value)

    x_roi = [convert(roi[0][0]), convert(roi[0][1])]
    y_roi = [convert(roi[1][0]), convert(roi[1][1])]
    z_roi = [convert(roi[2][0]), convert(roi[2][1])]

    return x_roi, y_roi, z_roi

def parse_roi(user_input):
    """
    Expected format:
        [[1, 5], [-2, 3], [-inf, 1]]
    """

    safe_input = (
        user_input
        .replace("-inf", "'-inf'")
        .replace("inf", "'inf'")
    )

    roi = ast.literal_eval(safe_input)

    if len(roi) != 3:
        raise ValueError("ROI must contain x, y and z bounds.")

    x_roi = [decode_bound(roi[0][0]), decode_bound(roi[0][1])]
    y_roi = [decode_bound(roi[1][0]), decode_bound(roi[1][1])]
    z_roi = [decode_bound(roi[2][0]), decode_bound(roi[2][1])]

    return x_roi, y_roi, z_roi


def crop_cloud(cloud, x_roi, y_roi, z_roi):
    xyz = cloud.points

    mask = (
        (xyz[:, 0] >= x_roi[0]) & (xyz[:, 0] <= x_roi[1]) &
        (xyz[:, 1] >= y_roi[0]) & (xyz[:, 1] <= y_roi[1]) &
        (xyz[:, 2] >= z_roi[0]) & (xyz[:, 2] <= z_roi[1])
    )

    return cloud.extract_points(mask, adjacent_cells=False)


def write_roi_to_json(config_path, x_roi, y_roi, z_roi):
    config_path = Path(config_path)

    with open(config_path, "r") as f:
        config = json.load(f)

    config["preprocessing"]["roi"] = encode_roi(x_roi, y_roi, z_roi)

    with open(config_path, "w") as f:
        json.dump(config, f, indent=4)

    print(f"Saved ROI to {config_path}")


def main():
    #ply_path = input("Enter path to .ply file: ").strip()
    #config_path = input("Enter path to JSON config file: ").strip()
    config_path = Path(__file__).parent.parent / "config" / "p3_config.json"
    ply_path = str(Path(__file__).parent.parent / "output" / "testout.vtp")
    cloud = pv.read(ply_path)
    print(cloud)
    print("\nShowing original cloud.")
    plotter = plotter_pcdisplay(cloud)

    while True:
        user_input = input(
            "\nEnter ROI bounds as [[xmin,xmax], [ymin,ymax], [zmin,zmax]]:\n"
            "Example: [[1,5], [-2,3], [-inf,1]]\n"
            "Type Q to quit.\n"
            "> "
        ).strip()

        # ----------------------------------------
        # Quit application
        # ----------------------------------------

        if user_input.upper() == "Q":
            plotter.close()
            del plotter

            print("Application closed.")
            break

        # ----------------------------------------
        # Parse ROI
        # ----------------------------------------

        try:
            x_roi, y_roi, z_roi = parse_roiNEW(user_input)

        except Exception as e:
            print(f"Invalid ROI input: {e}")
            continue

        # ----------------------------------------
        # Crop cloud
        # ----------------------------------------

        cropped = crop_cloud(cloud, x_roi, y_roi, z_roi)

        # ----------------------------------------
        # Replace display window
        # ----------------------------------------

        plotter.close()
        del plotter

        plotter = plotter_pcdisplay(cropped)

        print(f"Showing cropped cloud with {cropped.n_points} points.")

        # ----------------------------------------
        # Save / continue / quit
        # ----------------------------------------

        satisfied = input(
            "Are you satisfied with this ROI?\n"
            "Type Y to save, Q to quit, or ENTER to try again: "
        ).strip()

        if satisfied.upper() == "Q":
            plotter.close()
            del plotter

            print("Application closed.")
            break

        if satisfied.upper() == "Y":
            write_roi_to_json(config_path, x_roi, y_roi, z_roi)

            plotter.close()
            del plotter

            print("ROI saved.")
            break


if __name__ == "__main__":
    main()