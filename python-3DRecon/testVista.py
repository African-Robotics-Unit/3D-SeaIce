import pyvista as pv

ply_path = "totalCake.ply"

cloud = pv.read(ply_path)

plotter = pv.Plotter(window_size=(3000, 2200))
plotter.set_background("white")

plotter.add_mesh(
    cloud,
    style="points",
    point_size=3,
    render_points_as_spheres=True,
    color="black"
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

plotter.show(screenshot="pointcloud_publication.png")