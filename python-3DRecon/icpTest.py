import argparse
from pathlib import Path
import numpy as np
import open3d as o3d


def load_pcds(folder, voxel_size):
    paths = sorted(Path(folder).glob("*.ply"))
    if not paths:
        raise FileNotFoundError("No .ply files found.")

    pcds = []
    for p in paths:
        pc = o3d.io.read_point_cloud(str(p))
        if pc.is_empty():
            raise ValueError(f"Empty point cloud: {p}")

        pc = pc.voxel_down_sample(voxel_size)
        pc.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=voxel_size * 2,
                max_nn=30
            )
        )
        pcds.append(pc)

    return paths, pcds


def pairwise_registration(source, target, coarse_dist, fine_dist):
    print("Apply point-to-plane ICP")

    icp_coarse = o3d.pipelines.registration.registration_icp(
        source,
        target,
        coarse_dist,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )

    icp_fine = o3d.pipelines.registration.registration_icp(
        source,
        target,
        fine_dist,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )

    information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source,
        target,
        fine_dist,
        icp_fine.transformation
    )

    return icp_fine.transformation, information


def full_registration(pcds, coarse_dist, fine_dist):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.eye(4)

    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))

    for source_id in range(len(pcds)):
        for target_id in range(source_id + 1, len(pcds)):
            print(f"Registering cloud {source_id} -> {target_id}")

            transformation, information = pairwise_registration(
                pcds[source_id],
                pcds[target_id],
                coarse_dist,
                fine_dist
            )

            if target_id == source_id + 1:
                odometry = transformation @ odometry
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)
                    )
                )
                uncertain = False
            else:
                uncertain = True

            pose_graph.edges.append(
                o3d.pipelines.registration.PoseGraphEdge(
                    source_id,
                    target_id,
                    transformation,
                    information,
                    uncertain=uncertain
                )
            )

    return pose_graph


def optimize_pose_graph(pose_graph, fine_dist):
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=fine_dist,
        edge_prune_threshold=0.25,
        reference_node=0
    )

    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option
    )


def write_aln(filename, paths, poses):
    with open(filename, "w") as f:
        f.write(f"{len(poses)}\n")
        for path, T in zip(paths, poses):
            f.write(f"# {path.name}\n")
            for row in T:
                f.write(" ".join(f"{v:.10f}" for v in row) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", help="Folder containing .ply files")
    parser.add_argument("--out", default="transforms.aln")
    parser.add_argument("--voxel", type=float, default=0.02)
    parser.add_argument("--visualize", action="store_true")
    args = parser.parse_args()

    paths, pcds = load_pcds(args.folder, args.voxel)

    coarse_dist = args.voxel * 15
    fine_dist = args.voxel * 1.5

    pose_graph = full_registration(pcds, coarse_dist, fine_dist)
    optimize_pose_graph(pose_graph, fine_dist)

    poses = [node.pose for node in pose_graph.nodes]
    write_aln(args.out, paths, poses)

    print(f"Wrote {args.out}")

    if args.visualize:
        pcds_vis = []
        for pc, T in zip(pcds, poses):
            pc_copy = o3d.geometry.PointCloud(pc)
            pc_copy.transform(T)
            pcds_vis.append(pc_copy)

        o3d.visualization.draw_geometries(pcds_vis)


if __name__ == "__main__":
    main()