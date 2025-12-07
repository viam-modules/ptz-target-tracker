import json
import trimesh
import numpy as np
from datetime import datetime, timezone
import argparse
from pathlib import Path


def load_obstacles(obstacles_file):
    """Load obstacle bounding boxes from JSON file."""
    if not obstacles_file or not Path(obstacles_file).exists():
        return []
    
    with open(obstacles_file, 'r') as f:
        data = json.load(f)
    
    # Support both single obstacle and array of obstacles
    if isinstance(data, dict):
        obstacles = [data]
    else:
        obstacles = data
    
    parsed_obstacles = []
    for obs in obstacles:
        try:
            translation = obs['translation']
            geometry = obs['geometry']
            
            center = np.array([translation['x'], translation['y'], translation['z']])
            dims = np.array([geometry['x'], geometry['y'], geometry['z']])
            label = obs.get('label', 'obstacle')
            
            parsed_obstacles.append({
                'center': center,
                'dims': dims,
                'label': label
            })
        except KeyError as e:
            print(f"Warning: Skipping malformed obstacle (missing {e})")
            continue
    
    return parsed_obstacles


def check_collision_with_obstacles(pose_point, ee_length, ee_width, ee_depth, ee_clearance, obstacles):
    """
    Check if a pose would cause the end effector to collide with any obstacles.
    
    Args:
        pose_point: Position of end effector mounting point (x, y, z) in mm
        ee_length: Length of end effector extending downward in mm (Z dimension)
        ee_width: Width of end effector in mm (X dimension)
        ee_depth: Depth of end effector in mm (Y dimension)
        ee_clearance: Additional safety clearance around end effector in mm
        obstacles: List of obstacle dictionaries with 'center' and 'dims'
    
    Returns:
        tuple: (collides, obstacle_label) - True if collision, label of obstacle hit
    """
    if not obstacles:
        return False, None
    
    # End effector is a box from pose_point extending down by ee_length
    # Center of end effector box is at pose_point - [0, 0, ee_length/2]
    ee_center = pose_point - np.array([0, 0, ee_length / 2])
    
    # End effector dimensions (including clearance)
    ee_dims = np.array([
        ee_width + 2 * ee_clearance,
        ee_depth + 2 * ee_clearance,
        ee_length
    ])
    
    # End effector bounding box
    ee_min = ee_center - ee_dims / 2
    ee_max = ee_center + ee_dims / 2
    
    for obs in obstacles:
        obs_center = obs['center']
        obs_dims = obs['dims']
        
        # Obstacle bounding box
        obs_min = obs_center - obs_dims / 2
        obs_max = obs_center + obs_dims / 2
        
        # Check for AABB (Axis-Aligned Bounding Box) intersection
        # Two boxes collide if they overlap on ALL three axes
        x_overlap = ee_max[0] >= obs_min[0] and ee_min[0] <= obs_max[0]
        y_overlap = ee_max[1] >= obs_min[1] and ee_min[1] <= obs_max[1]
        z_overlap = ee_max[2] >= obs_min[2] and ee_min[2] <= obs_max[2]
        
        if x_overlap and y_overlap and z_overlap:
            return True, obs['label']
    
    return False, None


def main():
    # Parse command line arguments with named parameters
    parser = argparse.ArgumentParser(
        description='Generate calibration poses for PTZ camera tracking system.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  %(prog)s --num-poses 24 --safety-height 100
  %(prog)s -n 12 -s 50
  %(prog)s --num-poses 18 --safety-height 0 --reach 1700
        '''
    )
    
    parser.add_argument(
        '-n', '--num-poses',
        type=int,
        default=24,
        help='Number of poses to generate (12-24). Min 12: lower layer only. Max 24: full coverage (default: 24)'
    )
    
    parser.add_argument(
        '-s', '--safety-height',
        type=float,
        default=50.0,
        help='Minimum clearance above mesh top in mm (default: 50.0)'
    )
    
    parser.add_argument(
        '-r', '--reach',
        type=float,
        default=1700.0,
        help='Maximum reach of arm from origin in mm (default: 1700.0)'
    )
    
    parser.add_argument(
        '--arm-base-x',
        type=float,
        default=0.0,
        help='Arm base X position in mm (default: 0.0)'
    )
    
    parser.add_argument(
        '--arm-base-y',
        type=float,
        default=0.0,
        help='Arm base Y position in mm (default: 0.0)'
    )
    
    parser.add_argument(
        '--arm-base-z',
        type=float,
        default=0.0,
        help='Arm base Z position in mm (default: 0.0)'
    )
    
    parser.add_argument(
        '-m', '--mesh',
        type=str,
        default='mesh.ply',
        help='Path to mesh file (default: mesh.ply)'
    )
    
    parser.add_argument(
        '-o', '--output',
        type=str,
        default='ee_poses_generated.json',
        help='Output JSON file path (default: ee_poses_generated.json)'
    )
    
    parser.add_argument(
        '--obstacles',
        type=str,
        help='JSON file containing obstacle bounding boxes (optional)'
    )
    
    parser.add_argument(
        '--ee-length',
        type=float,
        default=0.0,
        help='End effector length extending below mounting point in mm (Z dimension, default: 0.0)'
    )
    
    parser.add_argument(
        '--ee-width',
        type=float,
        default=100.0,
        help='End effector width in mm (X dimension, default: 100.0)'
    )
    
    parser.add_argument(
        '--ee-depth',
        type=float,
        default=100.0,
        help='End effector depth in mm (Y dimension, default: 100.0)'
    )
    
    parser.add_argument(
        '--ee-clearance',
        type=float,
        default=50.0,
        help='Additional safety clearance around end effector in mm (default: 50.0)'
    )
    
    args = parser.parse_args()
    
    # Validate parameters
    num_poses_requested = args.num_poses
    if num_poses_requested < 12:
        print(f"Warning: Minimum is 12 poses. Setting to 12.")
        num_poses_requested = 12
    elif num_poses_requested > 24:
        print(f"Warning: Maximum is 24 poses. Setting to 24.")
        num_poses_requested = 24
    
    safety_height_mm = args.safety_height
    if safety_height_mm < 0:
        print(f"Warning: Safety height must be >= 0. Setting to 0.")
        safety_height_mm = 0.0
    
    MAX_REACH = args.reach
    if MAX_REACH <= 0:
        print(f"Warning: Reach must be > 0. Using default 1700mm.")
        MAX_REACH = 1700.0
    
    # Arm base position
    arm_base = np.array([args.arm_base_x, args.arm_base_y, args.arm_base_z])
    
    # Load obstacles
    obstacles = []
    if args.obstacles:
        obstacles = load_obstacles(args.obstacles)
        print(f"Loaded {len(obstacles)} obstacle(s) from {args.obstacles}")
        for obs in obstacles:
            print(f"  - {obs['label']}: center={obs['center']}, dims={obs['dims']}")
    
    print(f"\nConfiguration:")
    print(f"  Target number of poses: {num_poses_requested}")
    print(f"  Safety height above mesh: {safety_height_mm:.1f} mm")
    print(f"  Maximum arm reach: {MAX_REACH:.1f} mm")
    print(f"  Arm base position: [{arm_base[0]:.1f}, {arm_base[1]:.1f}, {arm_base[2]:.1f}] mm")
    print(f"  End effector dimensions: {args.ee_width:.1f} x {args.ee_depth:.1f} x {args.ee_length:.1f} mm (W x D x L)")
    print(f"  End effector clearance: {args.ee_clearance:.1f} mm")
    print(f"  Obstacles: {len(obstacles)}")
    print(f"  Mesh file: {args.mesh}")
    print(f"  Output file: {args.output}")
    print()
    
    # Load mesh
    try:
        mesh = trimesh.load(args.mesh)
        print(f"Loaded mesh with {len(mesh.vertices)} vertices")
    except Exception as e:
        print(f"Error: Could not load {args.mesh}: {e}")
        return
    
    # Get bounding box in meters (original mesh units)
    bounds = mesh.bounds
    min_corner = bounds[0]
    max_corner = bounds[1]
    
    print(f"\nMesh bounding box (meters):")
    print(f"  Min: [{min_corner[0]:.3f}, {min_corner[1]:.3f}, {min_corner[2]:.3f}]")
    print(f"  Max: [{max_corner[0]:.3f}, {max_corner[1]:.3f}, {max_corner[2]:.3f}]")
    
    # Convert to millimeters for pose generation
    min_mm = min_corner * 1000
    max_mm = max_corner * 1000
    
    print(f"\nMesh bounding box (millimeters):")
    print(f"  Min: [{min_mm[0]:.1f}, {min_mm[1]:.1f}, {min_mm[2]:.1f}]")
    print(f"  Max: [{max_mm[0]:.1f}, {max_mm[1]:.1f}, {max_mm[2]:.1f}]")
    
    # Generate poses ABOVE the mesh with better spatial coverage
    # Two layers: safety_height above mesh top, and 600mm above mesh top
    z_layers = [
        max_mm[2] + safety_height_mm,        # Lower layer: safety_height above mesh top (most important)
        max_mm[2] + 600                      # Upper layer: 600mm above mesh top
    ]
    
    print(f"\nGenerating poses ABOVE the mesh:")
    print(f"  Safety clearance: {safety_height_mm:.1f} mm above mesh top")
    for i, z in enumerate(z_layers, 1):
        print(f"  Layer {i} Z: {z:.1f} mm ({z - max_mm[2]:.1f} mm above mesh top)")
    print(f"  Mesh top is at {max_mm[2]:.1f} mm")
    
    # Determine grid size based on requested number of poses
    # Lower layer always gets 12 poses (3x4 grid)
    # If total > 12, add poses to upper layer
    nx_lower, ny_lower = 3, 4  # 12 poses on lower (most important) layer
    num_lower = nx_lower * ny_lower
    num_upper = num_poses_requested - num_lower
    
    print(f"\nPose distribution:")
    print(f"  Lower layer (most important): {num_lower} poses ({nx_lower}x{ny_lower} grid)")
    if num_upper > 0:
        # Calculate grid for upper layer to match requested count
        # Try to keep aspect ratio similar to lower layer
        if num_upper >= 12:
            nx_upper, ny_upper = 3, 4  # Full grid on upper layer
        else:
            # Distribute remaining poses as evenly as possible
            # Use simple factorization to get closest to square/rectangular grid
            best_nx, best_ny = 1, num_upper
            for nx in range(1, num_upper + 1):
                if num_upper % nx == 0:
                    ny = num_upper // nx
                    if abs(nx - ny) < abs(best_nx - best_ny):
                        best_nx, best_ny = nx, ny
            nx_upper, ny_upper = best_nx, best_ny
        print(f"  Upper layer: {num_upper} poses ({nx_upper}x{ny_upper} grid)")
    else:
        nx_upper, ny_upper = 0, 0
        print(f"  Upper layer: 0 poses (only lower layer)")
    
    poses = []
    
    # Calculate reasonable generation area based on reach and Z heights
    # The maximum XY distance we can have at each Z layer while staying within reach
    max_xy_radius_lower = np.sqrt(MAX_REACH**2 - (z_layers[0] - arm_base[2])**2) if MAX_REACH**2 > (z_layers[0] - arm_base[2])**2 else 0
    max_xy_radius_upper = np.sqrt(MAX_REACH**2 - (z_layers[1] - arm_base[2])**2) if MAX_REACH**2 > (z_layers[1] - arm_base[2])**2 else 0
    
    # Use 80% of max radius to ensure poses are comfortably within reach
    safe_margin = 0.8
    generation_radius_lower = max_xy_radius_lower * safe_margin
    generation_radius_upper = max_xy_radius_upper * safe_margin
    
    # Also constrain by mesh bounds
    x_min_gen = max(min_mm[0], arm_base[0] - generation_radius_lower)
    x_max_gen = min(max_mm[0], arm_base[0] + generation_radius_lower)
    y_min_gen = max(min_mm[1], arm_base[1] - generation_radius_lower)
    y_max_gen = min(max_mm[1], arm_base[1] + generation_radius_lower)
    
    print(f"\nGeneration area (constrained by reach and mesh):")
    print(f"  Max XY radius at lower layer: {max_xy_radius_lower:.1f} mm (using {safe_margin*100:.0f}% = {generation_radius_lower:.1f} mm)")
    print(f"  Max XY radius at upper layer: {max_xy_radius_upper:.1f} mm (using {safe_margin*100:.0f}% = {generation_radius_upper:.1f} mm)")
    print(f"  X: [{x_min_gen:.1f}, {x_max_gen:.1f}] mm")
    print(f"  Y: [{y_min_gen:.1f}, {y_max_gen:.1f}] mm")
    
    x_range = x_max_gen - x_min_gen
    y_range = y_max_gen - y_min_gen
    
    # Generate grid positions over the generation area (XY plane)
    # Add margins so poses aren't right at edges
    margin_factor = 0.1  # 10% margin from edges
    x_margin = x_range * margin_factor
    y_margin = y_range * margin_factor
    
    
    # Generate poses at grid intersections for each layer
    # Filter to only include poses within MAX_REACH from arm base position
    
    pose_count = 0
    rejected_count = 0
    rejected_by_obstacle = 0
    
    # Generate lower layer (always 12 poses)
    x_positions_lower = np.linspace(x_min_gen + x_margin, x_max_gen - x_margin, nx_lower)
    y_positions_lower = np.linspace(y_min_gen + y_margin, y_max_gen - y_margin, ny_lower)
    
    print(f"\nGenerating lower layer poses:")
    for y in y_positions_lower:
        for x in x_positions_lower:
            z = z_layers[0]
            # Calculate distance from arm base
            pose_pos = np.array([x, y, z])
            distance = np.linalg.norm(pose_pos - arm_base)
            
            # Skip poses that are beyond the arm's reach
            if distance > MAX_REACH:
                rejected_count += 1
                print(f"  ✗ Skipped (reach): [{x:.1f}, {y:.1f}, {z:.1f}] mm - Distance {distance:.1f} mm > {MAX_REACH} mm")
                continue
            
            # Check collision with obstacles
            collides, obstacle_label = check_collision_with_obstacles(
                pose_pos, args.ee_length, args.ee_width, args.ee_depth, args.ee_clearance, obstacles
            )
            if collides:
                rejected_by_obstacle += 1
                print(f"  ✗ Skipped (obstacle): [{x:.1f}, {y:.1f}, {z:.1f}] mm - Collision with {obstacle_label}")
                continue
            
            # Default orientation: pointing down (-Z direction)
            o_x = 1.0  # Rotate around X axis
            o_y = 0.0
            o_z = 0.0
            theta = 180.0  # 180 degrees to point down
            
            pose_data = {
                "data": {
                    "pose": {
                        "x": float(x),
                        "y": float(y),
                        "z": float(z),
                        "o_x": o_x,
                        "o_y": o_y,
                        "o_z": o_z,
                        "theta": theta
                    }
                },
                "tags": None,
                "additional_parameters": {},
                "method_name": "EndPosition",
                "organization_id": "generated",
                "time_requested": datetime.now(timezone.utc).isoformat(),
                "time_received": datetime.now(timezone.utc).isoformat(),
                "location_id": "generated",
                "robot_id": "generated",
                "part_id": "generated",
                "component_type": "rdk:component:arm",
                "component_name": "generated",
                "capture_day": datetime.now(timezone.utc).date().isoformat()
            }
            
            poses.append(pose_data)
            pose_count += 1
            
            height_above = z - max_mm[2]
            if height_above < 0:
                print(f"  ⚠️  Pose {pose_count}: [{x:.1f}, {y:.1f}, {z:.1f}] mm - BELOW MESH! Distance: {distance:.1f} mm")
            else:
                print(f"  ✓ Pose {pose_count}: [{x:.1f}, {y:.1f}, {z:.1f}] mm ({height_above:.0f}mm above mesh, distance: {distance:.1f} mm)")
    
    # Generate upper layer (if requested)
    if num_upper > 0:
        x_positions_upper = np.linspace(x_min_gen + x_margin, x_max_gen - x_margin, nx_upper)
        y_positions_upper = np.linspace(y_min_gen + y_margin, y_max_gen - y_margin, ny_upper)
        
        print(f"\nGenerating upper layer poses:")
        for y in y_positions_upper:
            for x in x_positions_upper:
                z = z_layers[1]
                # Calculate distance from arm base
                pose_pos = np.array([x, y, z])
                distance = np.linalg.norm(pose_pos - arm_base)
                
                # Skip poses that are beyond the arm's reach
                if distance > MAX_REACH:
                    rejected_count += 1
                    print(f"  ✗ Skipped (reach): [{x:.1f}, {y:.1f}, {z:.1f}] mm - Distance {distance:.1f} mm > {MAX_REACH} mm")
                    continue
                
                # Check collision with obstacles
                collides, obstacle_label = check_collision_with_obstacles(
                    pose_pos, args.ee_length, args.ee_width, args.ee_depth, args.ee_clearance, obstacles
                )
                if collides:
                    rejected_by_obstacle += 1
                    print(f"  ✗ Skipped (obstacle): [{x:.1f}, {y:.1f}, {z:.1f}] mm - Collision with {obstacle_label}")
                    continue
                
                # Default orientation: pointing down (-Z direction)
                o_x = 1.0
                o_y = 0.0
                o_z = 0.0
                theta = 180.0
                
                pose_data = {
                    "data": {
                        "pose": {
                            "x": float(x),
                            "y": float(y),
                            "z": float(z),
                            "o_x": o_x,
                            "o_y": o_y,
                            "o_z": o_z,
                            "theta": theta
                        }
                    },
                    "tags": None,
                    "additional_parameters": {},
                    "method_name": "EndPosition",
                    "organization_id": "generated",
                    "time_requested": datetime.now(timezone.utc).isoformat(),
                    "time_received": datetime.now(timezone.utc).isoformat(),
                    "location_id": "generated",
                    "robot_id": "generated",
                    "part_id": "generated",
                    "component_type": "rdk:component:arm",
                    "component_name": "generated",
                    "capture_day": datetime.now(timezone.utc).date().isoformat()
                }
                
                poses.append(pose_data)
                pose_count += 1
                
                print(f"  ✓ Pose {pose_count}: [{x:.1f}, {y:.1f}, {z:.1f}] mm (above mesh by {z - max_mm[2]:.1f} mm, distance: {distance:.1f} mm)")

    
    # Save to JSON file
    output_file = args.output
    with open(output_file, 'w') as f:
        json.dump(poses, f, indent=2)
    
    print(f"\n✅ Generated {len(poses)} poses (within {MAX_REACH} mm from arm base)")
    print(f"✗ Rejected {rejected_count} poses (beyond reach)")
    if rejected_by_obstacle > 0:
        print(f"✗ Rejected {rejected_by_obstacle} poses (obstacle collision)")
    print(f"✅ Saved to {output_file}")


if __name__ == "__main__":
    main()
