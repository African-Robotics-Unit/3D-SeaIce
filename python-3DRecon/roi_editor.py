

import json
from pathlib import Path

import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui


def decode_roi(values):
    out = []

    for v in values:
        if v == "inf":
            out.append(np.inf)
        elif v == "-inf":
            out.append(-np.inf)
        else:
            out.append(float(v))

    return out


def encode_roi(values):
    out = []

    for v in values:
        if np.isposinf(v):
            out.append("inf")
        elif np.isneginf(v):
            out.append("-inf")
        else:
            out.append(float(v))

    return out


class ROIEditor:

    def __init__(self, json_path, cloud_path):

        self.json_path = Path(json_path)
        self.cloud_path = Path(cloud_path)

        with open(self.json_path, "r") as f:
            self.config = json.load(f)

        self.pcd_original = o3d.io.read_point_cloud(str(self.cloud_path))

        if self.pcd_original.is_empty():
            raise ValueError("Point cloud could not be loaded.")

        self.points = np.asarray(self.pcd_original.points)

        roi = self.config.get("roi", {})

        self.x_roi = decode_roi(roi.get("x", [0, 7]))
        self.y_roi = decode_roi(roi.get("y", [-5, 5]))
        self.z_roi = decode_roi(roi.get("z", ["-inf", 2]))

        self.current_crop = self.pcd_original

        self.app = gui.Application.instance
        self.app.initialize()

        self.window = self.app.create_window(
            "ROI JSON Editor",
            400,
            400
        )

        self.panel = gui.Vert(
            10,
            gui.Margins(10, 10, 10, 10)
        )

        self.fields = {}

        for name, value in [
            ("x_min", self.x_roi[0]),
            ("x_max", self.x_roi[1]),
            ("y_min", self.y_roi[0]),
            ("y_max", self.y_roi[1]),
            ("z_min", self.z_roi[0]),
            ("z_max", self.z_roi[1]),
        ]:

            label = gui.Label(name)

            field = gui.TextEdit()
            field.text_value = self.num_to_text(value)

            self.fields[name] = field

            row = gui.Horiz()

            row.add_child(label)
            row.add_child(field)

            self.panel.add_child(row)

        update_button = gui.Button("Update Crop")
        update_button.set_on_clicked(self.update_crop)
        self.panel.add_child(update_button)

        view_button = gui.Button("Open 360 Viewer")
        view_button.set_on_clicked(self.open_360_viewer)
        self.panel.add_child(view_button)

        save_button = gui.Button("Save ROI to JSON")
        save_button.set_on_clicked(self.save_json)
        self.panel.add_child(save_button)

        self.status_label = gui.Label("Ready")
        self.panel.add_child(self.status_label)

        self.window.add_child(self.panel)

        self.update_crop()

    def num_to_text(self, x):

        if np.isposinf(x):
            return "inf"

        elif np.isneginf(x):
            return "-inf"

        else:
            return str(x)

    def text_to_num(self, text):

        text = text.strip().lower()

        if text in ["inf", "+inf"]:
            return np.inf

        elif text == "-inf":
            return -np.inf

        else:
            return float(text)

    def get_roi(self):

        return {
            "x": [
                self.text_to_num(
                    self.fields["x_min"].text_value
                ),
                self.text_to_num(
                    self.fields["x_max"].text_value
                ),
            ],

            "y": [
                self.text_to_num(
                    self.fields["y_min"].text_value
                ),
                self.text_to_num(
                    self.fields["y_max"].text_value
                ),
            ],

            "z": [
                self.text_to_num(
                    self.fields["z_min"].text_value
                ),
                self.text_to_num(
                    self.fields["z_max"].text_value
                ),
            ]
        }

    def get_cropped_cloud(self):

        roi = self.get_roi()

        pts = self.points

        mask = (
            (pts[:, 0] >= roi["x"][0]) &
            (pts[:, 0] <= roi["x"][1]) &

            (pts[:, 1] >= roi["y"][0]) &
            (pts[:, 1] <= roi["y"][1]) &

            (pts[:, 2] >= roi["z"][0]) &
            (pts[:, 2] <= roi["z"][1])
        )

        indices = np.where(mask)[0]

        cropped = self.pcd_original.select_by_index(indices)

        return cropped

    def update_crop(self):

        try:
            cropped = self.get_cropped_cloud()

        except Exception as e:

            self.status_label.text = f"Invalid ROI: {e}"
            return

        self.current_crop = cropped

        npts = len(cropped.points)

        self.status_label.text = (
            f"Crop updated: {npts} points"
        )

        print(f"Crop updated: {npts} points")

    def open_360_viewer(self):

        if self.current_crop.is_empty():

            self.status_label.text = (
                "Crop is empty"
            )

            return

        frame = (
            o3d.geometry.TriangleMesh
            .create_coordinate_frame(size=1.0)
        )

        o3d.visualization.draw_geometries(
            [self.current_crop, frame],
            window_name="360 ROI Viewer",
            width=1200,
            height=900
        )

    def save_json(self):

        roi = self.get_roi()

        self.config["roi"] = {
            "x": encode_roi(roi["x"]),
            "y": encode_roi(roi["y"]),
            "z": encode_roi(roi["z"]),
        }

        with open(self.json_path, "w") as f:

            json.dump(
                self.config,
                f,
                indent=4
            )

        self.status_label.text = (
            f"Saved to {self.json_path.name}"
        )

        print(f"Saved ROI to {self.json_path}")

    def run(self):

        self.app.run()

if __name__ == "__main__":
    ROOT = Path(__file__).resolve().parent.parent

    editor = ROIEditor(
        json_path="p3_config.json",
        cloud_path="totalCake.ply"
    )

    editor.run()