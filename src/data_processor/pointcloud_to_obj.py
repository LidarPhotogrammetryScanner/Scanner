import os
from pathlib import Path
import numpy as np
import open3d as o3d

def load_pcd(pcd_file):
    if not os.path.exists(pcd_file):
        raise FileNotFoundError(f"PCD file not found: {pcd_file}")
    
    pcd = o3d.io.read_point_cloud(pcd_file)
    if len(pcd.points) == 0:
        raise ValueError("Point cloud is empty")
    
    return pcd

def preprocess_pointcloud(
    pcd: o3d.geometry.PointCloud,
    remove_outliers: bool = True,
    outlier_nb_neighbors: int = 20,
    outlier_std_ratio: float = 2.0,
    voxel_size: float = 0.02
) -> o3d.geometry.PointCloud:
    # Remove statistical outliers
    if remove_outliers:
        pcd, outlier_indices = pcd.remove_statistical_outlier(
            nb_neighbors=outlier_nb_neighbors,
            std_ratio=outlier_std_ratio
        )
        print(f"Removed {len(outlier_indices)} outliers, {len(pcd.points)} points remaining")
    
    # Check if there are enough points remaining
    if len(pcd.points) < 3:
        raise ValueError("Not enough points remaining after preprocessing")
    
    # Downsample if the point cloud is too large
    if len(pcd.points) > 300000:
        print(f"Large point cloud detected ({len(pcd.points)} points). Downsampling to improve stability...")
        original_count = len(pcd.points)
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        print(f"Downsampled from {original_count} to {len(pcd.points)} points")
    
    # Estimate normals
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    )
    
    # Orient normals consistently, or else fallback to camera location
    try:
        pcd.orient_normals_consistent_tangent_plane(100)
    except RuntimeError as e:
        if "qhull" in str(e).lower() or "precision" in str(e).lower():
            pcd.orient_normals_towards_camera_location(camera_location=np.array([0.0, 0.0, 0.0]))
        else:
            raise
    
    return pcd


def reconstruct_surface(
    pcd: o3d.geometry.PointCloud,
    depth: int = 9,
    width: int = 0,
    scale: float = 1.1,
    linear_fit: bool = False
) -> tuple[o3d.geometry.TriangleMesh, np.ndarray]:
    # Perform Poisson surface reconstruction
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd,
        depth=depth,
        width=width,
        scale=scale,
        linear_fit=linear_fit
    )
    print(f"Reconstruction complete: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")
    
    if len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
        raise ValueError("Reconstruction produced empty mesh")
    
    return mesh, densities


def clean_mesh(
    mesh: o3d.geometry.TriangleMesh,
    densities: np.ndarray = None
) -> o3d.geometry.TriangleMesh:
    # Filter mesh by density
    if densities is not None and len(densities) > 0:
        vertices_to_remove = densities < np.quantile(densities, 0.01)
        mesh.remove_vertices_by_mask(vertices_to_remove)
    
    # Clean mesh
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    
    if len(mesh.triangles) == 0:
        raise ValueError("Mesh became empty after cleaning")
    
    # Compute mesh normals
    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()
    
    return mesh


def convert_to_obj(mesh: o3d.geometry.TriangleMesh, output_obj_path: str) -> bool:
    # Compute vertex normals for OBJ export
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    
    # Compute triangle normals for OBJ export
    if not mesh.has_triangle_normals():
        mesh.compute_triangle_normals()
    
    # Export to OBJ file
    success = o3d.io.write_triangle_mesh(str(output_obj_path), mesh)
    if success:
        return True
    else:
        return False


def pcd_to_obj(
    input_pcd_path: str,
    output_obj_path: str,
    depth: int = 9,
    width: int = 0,
    scale: float = 1.1,
    linear_fit: bool = False,
    remove_outliers: bool = True,
    outlier_nb_neighbors: int = 20,
    outlier_std_ratio: float = 2.0,
    voxel_size: float = 0.02
) -> bool:
    try:
        # Load PCD
        pcd = load_pcd(input_pcd_path)
        if pcd is None or len(pcd.points) == 0:
            print("Error: Failed to load point cloud or point cloud is empty")
            return False
        
        print(f"Loaded {len(pcd.points)} points")
        
        # Preprocess point cloud
        pcd = preprocess_pointcloud(
            pcd,
            remove_outliers=remove_outliers,
            outlier_nb_neighbors=outlier_nb_neighbors,
            outlier_std_ratio=outlier_std_ratio,
            voxel_size=voxel_size
        )
        
        # Reconstruct surface
        mesh, densities = reconstruct_surface(
            pcd,
            depth=depth,
            width=width,
            scale=scale,
            linear_fit=linear_fit
        )
        
        # Clean mesh
        mesh = clean_mesh(mesh, densities)
        
        # Convert to OBJ
        success = convert_to_obj(mesh, output_obj_path)
        
        if success:
            print(f"Successfully converted {input_pcd_path} to {output_obj_path}")
        else:
            print(f"Failed to export OBJ file")
        
        return success
        
    except ValueError as e:
        print(f"Error: {e}")
        return False
    
    except Exception as e:
        print(f"Unexpected error: {e}")
        return False