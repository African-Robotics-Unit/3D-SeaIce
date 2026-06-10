import pyvista as pv
import numpy as np
from pyvistaqt import BackgroundPlotter
import open3d as o3d


def choose_rounding_base(xyz):
    ranges = np.ptp(xyz, axis=0)   # max - min for x, y, z
    max_range = np.max(ranges)

    if max_range <= 5:
        return 0.5
    elif max_range <= 10:
        return 1
    elif max_range <= 50:
        return 5
    else:
        return 10


def rounded_bounds(xyz, base=10):
    """
    Generate PyVista bounds rounded outward to nearest base.

    Parameters
    ----------
    xyz : ndarray (N,3)
        Point coordinates

    base : int or float
        Rounding base (default 10)

    Returns
    -------
    bounds : list
        [xmin, xmax, ymin, ymax, zmin, zmax]
    """

    xmin = np.floor(np.min(xyz[:, 0]) / base) * base
    xmax = np.ceil(np.max(xyz[:, 0]) / base) * base

    ymin = np.floor(np.min(xyz[:, 1]) / base) * base
    ymax = np.ceil(np.max(xyz[:, 1]) / base) * base

    zmin = np.floor(np.min(xyz[:, 2]) / base) * base
    zmax = np.ceil(np.max(xyz[:, 2]) / base) * base

    return [xmin, xmax, ymin, ymax, zmin, zmax]


def show_pointcloud_intensity(
    cloud,
    screenshot_path=None,
    point_size=3,
    cmap="viridis",
    window_size=(560, 420),
):
    if isinstance(cloud, o3d.t.geometry.PointCloud):

        xyz = cloud.point["positions"].numpy()

        cloud_pv = pv.PolyData(xyz)

        if "intensity" in cloud.point:
            intensity = cloud.point["intensity"].numpy().flatten()
            cloud_pv["intensity"] = intensity

        cloud = cloud_pv

    try:
        plotter = pv.Plotter(window_size=window_size)

        plotter.set_background("white")

        plotter.add_mesh(
            cloud,
            scalars="intensity",
            cmap=cmap,
            style="points",
            point_size=point_size,
            render_points_as_spheres=True,
            scalar_bar_args={
                "title": "Intensity",
                "color": "black",
            }
        )
        mybase = choose_rounding_base(cloud.points)
        mybounds = rounded_bounds(cloud.points, base=mybase)

        plotter.show_grid(
            color="black",
            grid="back",
            location="outer",
            xtitle="X [m]",
            ytitle="Y [m]",
            ztitle="Z [m]",
            font_size=10,
            ticks="outside",
            bounds=mybounds
        )

        plotter.add_axes(
            xlabel="X",
            ylabel="Y",
            zlabel="Z",
            color="black"
        )

        plotter.view_yz(negative=True)
        plotter.camera.zoom(1.2)
        if screenshot_path is not None:
            plotter.show(screenshot=screenshot_path)
        else:
            plotter.show()
    finally:
        plotter.close()
        del plotter
        del cloud


def show_pointcloud_color(
    cloud,
    screenshot_path=None,
    point_size=3,
    window_size=(560, 420),
):
    """
    Display a point cloud with RGB colour (e.g. from a RealSense camera).

    Parameters
    ----------
    cloud : o3d.t.geometry.PointCloud or pv.PolyData
        Input cloud. If an o3d tensor cloud, must have 'positions' and
        optionally 'colors' (float32 [0, 1]) attributes.

    screenshot_path : str or None
        If provided, saves a screenshot to this path instead of
        opening an interactive window.

    point_size : int
        Rendered point size in pixels.

    window_size : tuple of int
        PyVista plotter window size (width, height) in pixels.
    """
    if isinstance(cloud, o3d.t.geometry.PointCloud):
        xyz = cloud.point["positions"].numpy()
        cloud_pv = pv.PolyData(xyz)
        if "colors" in cloud.point:
            colors = (cloud.point["colors"].numpy() * 255).astype(np.uint8)
            cloud_pv["colors"] = colors
        cloud = cloud_pv

    try:
        plotter = pv.Plotter(window_size=window_size)
        plotter.set_background("white")
        plotter.add_mesh(
            cloud,
            scalars="colors",
            rgb=True,
            style="points",
            point_size=point_size,
            render_points_as_spheres=True,
        )
        mybase = choose_rounding_base(cloud.points)
        mybounds = rounded_bounds(cloud.points, base=mybase)
        plotter.show_grid(
            color="black",
            grid="back",
            location="outer",
            xtitle="X [m]",
            ytitle="Y [m]",
            ztitle="Z [m]",
            font_size=10,
            ticks="outside",
            bounds=mybounds
        )
        plotter.add_axes(xlabel="X", ylabel="Y", zlabel="Z", color="black")
        plotter.view_yz(negative=True)
        plotter.camera.zoom(1.2)
        if screenshot_path is not None:
            plotter.show(screenshot=screenshot_path)
        else:
            plotter.show()
    finally:
        plotter.close()
        del plotter
        del cloud


def plotter_pcdisplay(
    cloud,
    screenshot_path=None,
    point_size=3,
    cmap="viridis",
    window_size=(560, 420),
):
    #plotter = pv.Plotter(window_size=window_size)
    plotter = BackgroundPlotter(window_size=window_size)
    plotter.set_background("white")
    if isinstance(cloud, o3d.t.geometry.PointCloud):
        xyz = cloud.point["positions"].numpy()

        pv_cloud = pv.PolyData(xyz)

        if "intensity" in cloud.point:
            intensity = cloud.point["intensity"].numpy().flatten()
            pv_cloud.point_data["intensity"] = intensity
        cloud=pv_cloud
    plotter.add_mesh(
        cloud,
        scalars="intensity",
        cmap=cmap,
        style="points",
        point_size=point_size,
        render_points_as_spheres=True,
        scalar_bar_args={
            "title": "Intensity",
            "color": "black",
        }
    )
    mybase = choose_rounding_base(cloud.points)
    mybounds = rounded_bounds(cloud.points, base=mybase)

    plotter.show_grid(
        color="black",
        grid="back",
        location="outer",
        xtitle="X [m]",
        ytitle="Y [m]",
        ztitle="Z [m]",
        font_size=10,
        ticks="outside",
        bounds=mybounds
    )

    plotter.add_axes(
        xlabel="X",
        ylabel="Y",
        zlabel="Z",
        color="black"
    )

    plotter.view_yz(negative=True)
    plotter.camera.zoom(1.2)
    #plotter.show(auto_close=False, interactive_update=True)
    return plotter
