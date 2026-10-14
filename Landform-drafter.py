"""
Developer: Python AI Assistant
Description: Optimized split topographic generator with 6 outputs:
             main terrain (mesh), primary pathways (mesh), secondary pathway surfaces (list of Patch Breps), 
             tertiary pathway surfaces (list of Patch Breps), edge pathway surfaces (list of Patch Breps), 
             and programmed area surfaces (list of Patch Breps).
Inputs:
    edges: List of curves representing the boundary (perimeter).
    ridges: List of curves representing internal ridge lines.
    paths: List of closed curves representing primary road/walkway boundaries.
    second_paths: List of closed curves representing secondary walkway/trail boundaries.
    third_paths: List of closed curves representing tertiary/maintenance pathway boundaries.
    edge_paths: List of closed curves representing edge pathway boundaries.
    programmed_area: List of closed curves representing programmed zone boundaries.
    sample_dist: Float value representing the distance interval to sample points.
    smoothness: Float value (0.0 to 1.0) representing the terrain smoothness.
    patch_uv_spans: Integer value controlling equal U and V surface span counts for patch surfaces.
Outputs:
    topo_mesh: Clean main terrain mesh.
    path_mesh: Clean rebuilt primary pathway mesh.
    second_path_surf: List of Patch Surface Breps representing individual secondary pathways.
    third_path_surf: List of Patch Surface Breps representing individual tertiary pathways.
    edge_path_surf: List of Patch Surface Breps representing individual edge pathways.
    programmed_surf: List of Patch Surface Breps representing individual programmed zones.
"""

import Rhino
import Rhino.Geometry as rg

def extract_rhino_curve(gh_curve_input):
    """Safely unwraps Grasshopper geometric objects to Rhino.Geometry.Curve."""
    if gh_curve_input is None:
        return None
    if hasattr(gh_curve_input, "Value"):
        gh_curve_input = gh_curve_input.Value
    if isinstance(gh_curve_input, rg.Curve):
        return gh_curve_input
    if hasattr(gh_curve_input, "ToNurbsCurve"):
        return gh_curve_input.ToNurbsCurve()
    return None

def flatten_input(input_data):
    """Flattens input structures recursively."""
    if input_data is None:
        return []
    if isinstance(input_data, (list, tuple)):
        result = []
        for item in input_data:
            if isinstance(item, (list, tuple)):
                result.extend(flatten_input(item))
            else:
                result.append(item)
        return result
    return [input_data]

def process_and_project_curves(curve_list, plane):
    """Converts inputs to valid 2D flat curves and caches 2D bounding boxes with safety margins."""
    processed = []
    for item in curve_list:
        crv = extract_rhino_curve(item)
        if crv and crv.IsValid:
            flat_crv = rg.Curve.ProjectToPlane(crv, plane)
            target = flat_crv if flat_crv is not None else crv
            bbox = target.GetBoundingBox(True)
            bbox.Inflate(0.01)
            processed.append((crv, target, bbox))
    return processed

def sample_curve_points_fast(processed_curves, division_dist):
    """Samples 3D points efficiently along curves."""
    pts = []
    min_dist = max(0.001, division_dist)
    for crv_3d, _, _ in processed_curves:
        t_params = crv_3d.DivideByLength(min_dist, True)
        if t_params:
            pts.extend([crv_3d.PointAt(t) for t in t_params])
        else:
            pts.append(crv_3d.PointAtStart)
            pts.append(crv_3d.PointAtEnd)
    return pts

def point_in_curves(pt, processed_curves, plane, tol):
    """Fast containment check: Bounding Box pre-filtering before executing Curve.Contains."""
    x, y = pt.X, pt.Y
    for _, crv_2d, bbox in processed_curves:
        if bbox.Min.X <= x <= bbox.Max.X and bbox.Min.Y <= y <= bbox.Max.Y:
            if crv_2d.IsClosed:
                rel = crv_2d.Contains(pt, plane, tol)
                if rel in (rg.PointContainment.Inside, rg.PointContainment.Coincident):
                    return True
    return False

def face_center_in_curves(mesh, face_idx, processed_curves, plane, tol):
    """Exact centroid check to strictly classify face ownership."""
    f_center = mesh.Faces.GetFaceCenter(face_idx)
    f_center_2d = rg.Point3d(f_center.X, f_center.Y, 0.0)
    return point_in_curves(f_center_2d, processed_curves, plane, tol)

def clean_and_reconstruct_mesh(parent_mesh, face_mask):
    """Sub-mesh reconstruction using vertex index mapping."""
    if parent_mesh is None or not any(face_mask):
        return None

    new_mesh = rg.Mesh()
    parent_verts = parent_mesh.Vertices
    parent_faces = parent_mesh.Faces
    
    vertex_map = {}

    for i, include in enumerate(face_mask):
        if not include:
            continue

        face = parent_faces[i]
        old_indices = (face.A, face.B, face.C, face.D) if face.IsQuad else (face.A, face.B, face.C)
        
        new_indices = []
        for old_idx in old_indices:
            if old_idx not in vertex_map:
                new_idx = new_mesh.Vertices.Add(parent_verts[old_idx])
                vertex_map[old_idx] = new_idx
                new_indices.append(new_idx)
            else:
                new_indices.append(vertex_map[old_idx])

        if len(new_indices) == 3:
            new_mesh.Faces.AddFace(new_indices[0], new_indices[1], new_indices[2])
        else:
            new_mesh.Faces.AddFace(new_indices[0], new_indices[1], new_indices[2], new_indices[3])

    new_mesh.Faces.CullDegenerateFaces()
    new_mesh.Vertices.CullUnused()
    new_mesh.Compact()
    return new_mesh

def generate_individual_patch_surfaces(parent_mesh, zone_curves, plane, spans=10):
    """Generates individual Patch Surface Breps for each closed curve in zone_curves using equal U and V spans."""
    if parent_mesh is None or not zone_curves:
        return []

    doc = Rhino.RhinoDoc.ActiveDoc
    tol = doc.ModelAbsoluteTolerance if doc else 0.001
    patch_surfaces = []

    parent_verts = parent_mesh.Vertices
    parent_faces = parent_mesh.Faces
    
    # Ensure span count is valid (minimum 1)
    spans_count = max(1, int(spans))

    for crv_3d, crv_2d, bbox in zone_curves:
        # Find vertices strictly inside or coincident with this specific curve
        curve_verts = set()
        for i in range(parent_mesh.Faces.Count):
            f_center = parent_mesh.Faces.GetFaceCenter(i)
            f_center_2d = rg.Point3d(f_center.X, f_center.Y, 0.0)

            if bbox.Min.X <= f_center_2d.X <= bbox.Max.X and bbox.Min.Y <= f_center_2d.Y <= bbox.Max.Y:
                rel = crv_2d.Contains(f_center_2d, plane, tol)
                if rel in (rg.PointContainment.Inside, rg.PointContainment.Coincident):
                    face = parent_faces[i]
                    curve_verts.add(face.A)
                    curve_verts.add(face.B)
                    curve_verts.add(face.C)
                    if face.IsQuad:
                        curve_verts.add(face.D)

        # Assemble patch input geometry for this individual curve
        patch_inputs = [crv_3d]
        if curve_verts:
            pt_cloud = rg.PointCloud([parent_verts[idx] for idx in curve_verts])
            patch_inputs.append(pt_cloud)

        # Generate Patch Surface with equal U and V spans
        patch_brep = rg.Brep.CreatePatch(patch_inputs, spans_count, spans_count, tol)
        if patch_brep and patch_brep.IsValid:
            patch_surfaces.append(patch_brep)

    return patch_surfaces

def create_topography_optimized(edge_curves, ridge_curves, path_curves, second_path_curves, third_path_curves, edge_path_curves, prog_curves, division_dist, smooth_factor, spans_val):
    doc = Rhino.RhinoDoc.ActiveDoc
    tol = doc.ModelAbsoluteTolerance if doc else 0.001
    plane = rg.Plane.WorldXY

    if division_dist <= 0:
        division_dist = 1.0

    # 1. Process and Cache Curves
    edges_proc = process_and_project_curves(edge_curves, plane)
    if not edges_proc:
        print("Error: No valid edge curves provided.")
        return None, None, [], [], [], []

    # Outer Boundary Handling
    rhino_edges_3d = [item[0] for item in edges_proc]
    joined_edges = rg.Curve.JoinCurves(rhino_edges_3d, tol)
    if not joined_edges:
        print("Error: Boundary curves could not be joined.")
        return None, None, [], [], [], []
    
    boundary_proc = []
    for crv in joined_edges:
        crv_2d = rg.Curve.ProjectToPlane(crv, plane) or crv
        bbox = crv_2d.GetBoundingBox(True)
        bbox.Inflate(0.01)
        boundary_proc.append((crv, crv_2d, bbox))

    ridges_proc = process_and_project_curves(ridge_curves, plane)
    paths_proc = process_and_project_curves(path_curves, plane)
    second_paths_proc = process_and_project_curves(second_path_curves, plane)
    third_paths_proc = process_and_project_curves(third_path_curves, plane)
    edge_paths_proc = process_and_project_curves(edge_path_curves, plane)
    prog_proc = process_and_project_curves(prog_curves, plane)

    # 2. Fast Point Sampling
    all_points = []
    all_points.extend(sample_curve_points_fast(edges_proc, division_dist))
    all_points.extend(sample_curve_points_fast(ridges_proc, division_dist))
    all_points.extend(sample_curve_points_fast(paths_proc, division_dist * 0.5))
    all_points.extend(sample_curve_points_fast(second_paths_proc, division_dist * 0.5))
    all_points.extend(sample_curve_points_fast(third_paths_proc, division_dist * 0.5))
    all_points.extend(sample_curve_points_fast(edge_paths_proc, division_dist * 0.5))
    all_points.extend(sample_curve_points_fast(prog_proc, division_dist * 0.5))

    if len(all_points) < 3:
        print("Error: Insufficient points generated.")
        return None, None, [], [], [], []

    # 3. Create Master Mesh
    unified_mesh = rg.Mesh.CreateFromTessellation(all_points, None, plane, False)
    if not unified_mesh or unified_mesh.Faces.Count == 0:
        print("Error: Mesh tessellation failed.")
        return None, None, [], [], [], []

    # 4. Strict Exclusive Face Classification
    num_faces = unified_mesh.Faces.Count
    terrain_face_mask = [False] * num_faces
    path_face_mask = [False] * num_faces

    for i in range(num_faces):
        # Mandatory Boundary Check
        if not face_center_in_curves(unified_mesh, i, boundary_proc, plane, tol):
            continue

        # Exclusive Priority Waterfall: Patch areas are excluded from mesh face retention
        if face_center_in_curves(unified_mesh, i, prog_proc, plane, tol):
            continue
        elif face_center_in_curves(unified_mesh, i, second_paths_proc, plane, tol):
            continue
        elif face_center_in_curves(unified_mesh, i, third_paths_proc, plane, tol):
            continue
        elif face_center_in_curves(unified_mesh, i, edge_paths_proc, plane, tol):
            continue
        elif face_center_in_curves(unified_mesh, i, paths_proc, plane, tol):
            path_face_mask[i] = True
        else:
            terrain_face_mask[i] = True

    # 5. Extract Sub-Meshes for terrain and primary path
    terrain_mesh = clean_and_reconstruct_mesh(unified_mesh, terrain_face_mask)
    path_mesh = clean_and_reconstruct_mesh(unified_mesh, path_face_mask)

    # 6. Generate Individual Patch Surfaces (Breps)
    second_path_surf = generate_individual_patch_surfaces(unified_mesh, second_paths_proc, plane, spans_val)
    third_path_surf = generate_individual_patch_surfaces(unified_mesh, third_paths_proc, plane, spans_val)
    edge_path_surf = generate_individual_patch_surfaces(unified_mesh, edge_paths_proc, plane, spans_val)
    programmed_surf = generate_individual_patch_surfaces(unified_mesh, prog_proc, plane, spans_val)

    # 7. Post-split Smoothing & Weld for Mesh outputs
    weld_angle = 3.14159  # 180 degrees in radians

    meshes_and_smoothness = [
        (terrain_mesh, int(smooth_factor * 15), 0.5),
        (path_mesh, 8, 0.3)
    ]

    for m, iterations, step_size in meshes_and_smoothness:
        if m and iterations > 0:
            m.Smooth(step_size, iterations, True, True, False, True, rg.SmoothingCoordinateSystem.World, plane)
            m.Weld(weld_angle)
            m.Normals.ComputeNormals()

    return terrain_mesh, path_mesh, second_path_surf, third_path_surf, edge_path_surf, programmed_surf

# Grasshopper Entry Point
if __name__ == "__main__":
    edge_list = flatten_input(edges) if 'edges' in locals() else []
    ridge_list = flatten_input(ridges) if 'ridges' in locals() else []
    path_list = flatten_input(paths) if 'paths' in locals() else []
    second_path_list = flatten_input(second_paths) if 'second_paths' in locals() else []
    third_path_list = flatten_input(third_paths) if 'third_paths' in locals() else []
    edge_path_list = flatten_input(edge_paths) if 'edge_paths' in locals() else []
    prog_list = flatten_input(programmed_area) if 'programmed_area' in locals() else []
    
    dist_val = sample_dist if ('sample_dist' in locals() and sample_dist is not None) else 1.0
    smooth_val = float(smoothness) if ('smoothness' in locals() and smoothness is not None) else 0.0
    spans_val = int(patch_uv_spans) if ('patch_uv_spans' in locals() and patch_uv_spans is not None) else 10
    
    out_topo, out_path, out_second_surf, out_third_surf, out_edge_surf, out_prog_surf = create_topography_optimized(
        edge_list, ridge_list, path_list, second_path_list, third_path_list, edge_path_list, prog_list, dist_val, smooth_val, spans_val
    )
    
    topo_mesh = out_topo
    path_mesh = out_path
    second_path_surf = out_second_surf
    third_path_surf = out_third_surf
    edge_path_surf = out_edge_surf
    programmed_surf = out_prog_surf
