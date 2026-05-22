import pyvista as pv


def show_pointcloud_intensity(
    cloud,
    screenshot_path=None,
    point_size=3,
    cmap="viridis",
    window_size=(3000, 2200),
):

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

    plotter.show_grid(
        color="black",
        grid="back",
        location="outer",
        xlabel="X [m]",
        ylabel="Y [m]",
        zlabel="Z [m]",
        font_size=12
    )

    plotter.add_axes(
        xlabel="X",
        ylabel="Y",
        zlabel="Z",
        color="black"
    )

    plotter.camera_position = "iso"
    plotter.camera.zoom(1.2)

    if screenshot_path is not None:
        plotter.show(screenshot=screenshot_path)
    else:
        plotter.show()