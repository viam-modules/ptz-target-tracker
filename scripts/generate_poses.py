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


def find_highest_obstacle_beneath(x, y, ee_x, ee_y, ee_z, ee_clearance, obstacles):
    """
    Find the highest obstacle that would be beneath the end effector at position (x, y).
    
    Args:
        x, y: XY position of the pose
        ee_x: Size of end effector in mm (X dimension)
        ee_y: Size of end effector in mm (Y dimension)
        ee_z: Size of end effector extending downward in mm (Z dimension)
        ee_clearance: Additional safety clearance around end effector in mm
        obstacles: List of obstacle dictionaries with 'center' and 'dims'
    
    Returns:
        tuple: (adjusted_pose_z, obstacle_label) - Z coordinate where pose should be placed so 
               end effector bottom sits at obstacle top + clearance, or (None, None)
    """
    if not obstacles:
        return None, None
    
    # End effector footprint in XY (with clearance)
    ee_x_with_clearance = ee_x + 2 * ee_clearance
    ee_y_with_clearance = ee_y + 2 * ee_clearance
    
    highest_obs_top_z = None
    highest_label = None
    
    for obs in obstacles:
        obs_center = obs['center']
        obs_dims = obs['dims']
        
        # Obstacle bounding box
        obs_min = obs_center - obs_dims / 2
        obs_max = obs_center + obs_dims / 2
        
        # Check if end effector footprint (centered at x, y) overlaps with obstacle in XY plane
        ee_min_x = x - ee_x_with_clearance / 2
        ee_max_x = x + ee_x_with_clearance / 2
        ee_min_y = y - ee_y_with_clearance / 2
        ee_max_y = y + ee_y_with_clearance / 2
        
        x_overlap = ee_max_x >= obs_min[0] and ee_min_x <= obs_max[0]
        y_overlap = ee_max_y >= obs_min[1] and ee_min_y <= obs_max[1]
        
        if x_overlap and y_overlap:
            # This obstacle is beneath the end effector
            obs_top_z = obs_max[2]  # Top of obstacle
            if highest_obs_top_z is None or obs_top_z > highest_obs_top_z:
                highest_obs_top_z = obs_top_z
                highest_label = obs['label']
    
    if highest_obs_top_z is not None:
        # Calculate pose Z so that:
        # - End effector extends from pose_z down to (pose_z - ee_z)
        # - Bottom of end effector (pose_z - ee_z) should be at obstacle_top + clearance
        # Therefore: pose_z - ee_z = obstacle_top + clearance
        # So: pose_z = obstacle_top + clearance + ee_z
        adjusted_pose_z = highest_obs_top_z + ee_clearance + ee_z
        return adjusted_pose_z, highest_label
    
    return None, None


def check_collision_with_obstacles(pose_point, ee_z, ee_x, ee_y, ee_clearance, obstacles):
    """
    Check if a pose would cause the end effector to collide with any obstacles.
    
    Args:
        pose_point: Position of end effector mounting point (x, y, z) in mm
        ee_z: Size of end effector extending downward in mm (Z dimension)
        ee_x: Size of end effector in mm (X dimension)
        ee_y: Size of end effector in mm (Y dimension)
        ee_clearance: Additional safety clearance around end effector in mm
        obstacles: List of obstacle dictionaries with 'center' and 'dims'
    
    Returns:
        tuple: (collides, obstacle_label) - True if collision, label of obstacle hit
    """
    if not obstacles:
        return False, None
    
    # End effector is a box from pose_point extending down by ee_z
    # Center of end effector box is at pose_point - [0, 0, ee_z/2]
    ee_center = pose_point - np.array([0, 0, ee_z / 2])
    
    # End effector dimensions (including clearance)
    ee_dims = np.array([
        ee_x + 2 * ee_clearance,
        ee_y + 2 * ee_clearance,
        ee_z
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
  %(prog)s --num-poses 24 --ee-clearance 50
  %(prog)s -n 12 --ee-clearance 30
  %(prog)s --num-poses 18 --ee-clearance 100 --reach 1700
        '''
    )
    
    parser.add_argument(
        '-n', '--num-poses',
        type=int,
        default=24,
        help='Number of poses to generate (9-17). Min 9: lower layer only. Max 17: lower + high layer + 4 fully extended (default: 24)'
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
        '--ee-x',
        type=float,
        default=100.0,
        help='End effector size in X dimension in mm (default: 100.0)'
    )
    
    parser.add_argument(
        '--ee-y',
        type=float,
        default=100.0,
        help='End effector size in Y dimension in mm (default: 100.0)'
    )
    
    parser.add_argument(
        '--ee-z',
        type=float,
        default=0.0,
        help='End effector size in Z dimension (extending below mounting point) in mm (default: 0.0)'
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
    if num_poses_requested < 9:
        print(f"Warning: Minimum is 9 poses. Setting to 9.")
        num_poses_requested = 9
    elif num_poses_requested > 17:
        print(f"Warning: Maximum is 17 poses. Setting to 17.")
        num_poses_requested = 17
    
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
    print(f"  Maximum arm reach: {MAX_REACH:.1f} mm")
    print(f"  Arm base position: [{arm_base[0]:.1f}, {arm_base[1]:.1f}, {arm_base[2]:.1f}] mm")
    print(f"  End effector dimensions: {args.ee_x:.1f} x {args.ee_y:.1f} x {args.ee_z:.1f} mm (X x Y x Z)")
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
    # Three layers: ee_clearance above mesh top, 600mm, and 1000mm above mesh top
    z_layers = [
        max_mm[2] + args.ee_clearance,       # Lower layer: ee_clearance above mesh top (most important)
        max_mm[2] + 600,                     # Middle layer: 600mm above mesh top
        max_mm[2] + 1000                     # High layer: 1000mm above mesh top (maximum reach)
    ]
    
    print(f"\nGenerating poses ABOVE the mesh:")
    print(f"  Clearance: {args.ee_clearance:.1f} mm above mesh top")
    for i, z in enumerate(z_layers, 1):
        print(f"  Layer {i} Z: {z:.1f} mm ({z - max_mm[2]:.1f} mm above mesh top)")
    print(f"  Mesh top is at {max_mm[2]:.1f} mm")
    
    # Determine grid size based on requested number of poses
    # Lower layer always gets 9 poses (3x3 grid)
    # High layer gets remaining poses (3x1 grid + optional extra far pose)
    nx_lower, ny_lower = 3, 3  # 9 poses on lower (most important) layer
    num_lower = nx_lower * ny_lower
    
    remaining = num_poses_requested - num_lower
    if remaining > 0:
        num_high = min(remaining, 3)  # Standard 3x1 grid
        nx_high, ny_high = 3, 1
        num_high_extra = remaining - 3 if remaining > 3 else 0  # Extra far pose
    else:
        num_high = 0
        nx_high, ny_high = 0, 0
        num_high_extra = 0
    
    # No middle layer
    num_middle = 0
    nx_middle, ny_middle = 0, 0
    
    print(f"\nPose distribution:")
    print(f"  Lower layer (most important): {num_lower} poses ({nx_lower}x{ny_lower} grid)")
    print(f"  Middle layer: 0 poses")
    if num_high > 0:
        print(f"  High layer (max reach): {num_high} poses ({nx_high}x{ny_high} grid)")
    else:
        print(f"  High layer: 0 poses")
    
    poses = []
    
    # Calculate reasonable generation area based on reach and Z heights
    # The maximum XY distance we can have at each Z layer while staying within reach
    max_xy_radius_lower = np.sqrt(MAX_REACH**2 - (z_layers[0] - arm_base[2])**2) if MAX_REACH**2 > (z_layers[0] - arm_base[2])**2 else 0
    max_xy_radius_middle = np.sqrt(MAX_REACH**2 - (z_layers[1] - arm_base[2])**2) if MAX_REACH**2 > (z_layers[1] - arm_base[2])**2 else 0
    max_xy_radius_high = np.sqrt(MAX_REACH**2 - (z_layers[2] - arm_base[2])**2) if MAX_REACH**2 > (z_layers[2] - arm_base[2])**2 else 0
    
    # Use 80% of max radius to ensure poses are comfortably within reach
    safe_margin = 0.8
    generation_radius_lower = max_xy_radius_lower * safe_margin
    generation_radius_middle = max_xy_radius_middle * safe_margin
    generation_radius_high = max_xy_radius_high * safe_margin
    
    # Also constrain by mesh bounds
    x_min_gen = max(min_mm[0], arm_base[0] - generation_radius_lower)
    x_max_gen = min(max_mm[0], arm_base[0] + generation_radius_lower)
    y_min_gen = max(min_mm[1], arm_base[1] - generation_radius_lower)
    y_max_gen = min(max_mm[1], arm_base[1] + generation_radius_lower)
    
    print(f"\nGeneration area (constrained by reach and mesh):")
    print(f"  Max XY radius at lower layer: {max_xy_radius_lower:.1f} mm (using {safe_margin*100:.0f}% = {generation_radius_lower:.1f} mm)")
    print(f"  Max XY radius at middle layer: {max_xy_radius_middle:.1f} mm (using {safe_margin*100:.0f}% = {generation_radius_middle:.1f} mm)")
    print(f"  Max XY radius at high layer: {max_xy_radius_high:.1f} mm (using {safe_margin*100:.0f}% = {generation_radius_high:.1f} mm)")
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
            z = z_layers[0]  # Start with default lower layer Z
            
            # Check if there's an obstacle beneath this XY position
            obstacle_adjusted_z, obstacle_label = find_highest_obstacle_beneath(
                x, y, args.ee_x, args.ee_y, args.ee_z, args.ee_clearance, obstacles
            )
            
            if obstacle_adjusted_z is not None:
                # Use adjusted Z so end effector bottom sits on top of obstacle (with clearance)
                print(f"  ⚠ Adjusted Z for obstacle '{obstacle_label}': [{x:.1f}, {y:.1f}] from {z:.1f} to {obstacle_adjusted_z:.1f} mm")
                z = obstacle_adjusted_z
            
            # Calculate distance from arm base
            pose_pos = np.array([x, y, z])
            distance = np.linalg.norm(pose_pos - arm_base)
            
            # Skip poses that are beyond the arm's reach
            if distance > MAX_REACH:
                rejected_count += 1
                print(f"  ✗ Skipped (reach): [{x:.1f}, {y:.1f}, {z:.1f}] mm - Distance {distance:.1f} mm > {MAX_REACH} mm")
                continue
            
            # Final collision check (should pass now, but verify)
            collides, collision_label = check_collision_with_obstacles(
                pose_pos, args.ee_z, args.ee_x, args.ee_y, args.ee_clearance, obstacles
            )
            if collides:
                rejected_by_obstacle += 1
                print(f"  ✗ Skipped (obstacle): [{x:.1f}, {y:.1f}, {z:.1f}] mm - Collision with {collision_label}")
                continue
            
            # Default orientation: pointing down (-Z direction)
            o_x = 0.0  # Rotate around X axis
            o_y = 0.0
            o_z = -1.0
            theta = 90.0  # 180 degrees to point down
            
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
    
    # Generate middle layer (if requested)
    if num_middle > 0:
        x_positions_middle = np.linspace(x_min_gen + x_margin, x_max_gen - x_margin, nx_middle)
        y_positions_middle = np.linspace(y_min_gen + y_margin, y_max_gen - y_margin, ny_middle)
        
        print(f"\nGenerating middle layer poses:")
        for y in y_positions_middle:
            for x in x_positions_middle:
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
                    pose_pos, args.ee_z, args.ee_x, args.ee_y, args.ee_clearance, obstacles
                )
                if collides:
                    rejected_by_obstacle += 1
                    print(f"  ✗ Skipped (obstacle): [{x:.1f}, {y:.1f}, {z:.1f}] mm - Collision with {obstacle_label}")
                    continue
                
                # Default orientation: pointing down (-Z direction)
                o_x = 0.0
                o_y = 0.0
                o_z = -1.0
                theta = 90.0
                
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
                print(f"  ✓ Pose {pose_count}: [{x:.1f}, {y:.1f}, {z:.1f}] mm ({height_above:.0f}mm above mesh, distance: {distance:.1f} mm)")
    
    # Generate high layer (if requested)
    if num_high > 0:
        # Use reduced range for high layer to stay within reach
        # Calculate tighter bounds based on max_xy_radius_high
        x_high_limit = min(max_xy_radius_high, x_max_gen - x_margin)
        y_high_limit = min(max_xy_radius_high * 0.7, y_max_gen - y_margin)  # More conservative on Y due to offset
        x_positions_high = np.linspace(-x_high_limit, x_high_limit, nx_high)
        y_positions_high = np.linspace(y_min_gen + y_margin, y_high_limit, ny_high)
        
        print(f"\nGenerating high layer poses:")
        for y in y_positions_high:
            for x in x_positions_high:
                z = z_layers[2]
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
                    pose_pos, args.ee_z, args.ee_x, args.ee_y, args.ee_clearance, obstacles
                )
                if collides:
                    rejected_by_obstacle += 1
                    print(f"  ✗ Skipped (obstacle): [{x:.1f}, {y:.1f}, {z:.1f}] mm - Collision with {obstacle_label}")
                    continue
                
                # Default orientation: pointing down (-Z direction)
                o_x = 0.0
                o_y = 0.0
                o_z = -1.0
                theta = 90.0
                
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
                print(f"  ✓ Pose {pose_count}: [{x:.1f}, {y:.1f}, {z:.1f}] mm ({height_above:.0f}mm above mesh, distance: {distance:.1f} mm)")
    
    # Add extra high pose at maximum Y reach (if requested)
    if num_high_extra > 0:
        z = z_layers[2]
        # Calculate maximum Y at this Z while staying within MAX_REACH
        # Using: distance^2 = x^2 + y^2 + z^2, with x=0
        max_y_at_high_z = np.sqrt(MAX_REACH**2 - z**2)
        
        x = 0.0
        y = max_y_at_high_z
        pose_pos = np.array([x, y, z])
        distance = np.linalg.norm(pose_pos - arm_base)
        
        # Check collision with obstacles
        collides, obstacle_label = check_collision_with_obstacles(
            pose_pos, args.ee_z, args.ee_x, args.ee_y, args.ee_clearance, obstacles
        )
        
        if not collides and distance <= MAX_REACH:
            o_x = 0.0
            o_y = 0.0
            o_z = -1.0
            theta = 90.0
            
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
            print(f"  ✓ Pose {pose_count} (far reach): [{x:.1f}, {y:.1f}, {z:.1f}] mm ({height_above:.0f}mm above mesh, distance: {distance:.1f} mm)")
        else:
            if collides:
                rejected_by_obstacle += 1
                print(f"  ✗ Skipped far pose (obstacle): [{x:.1f}, {y:.1f}, {z:.1f}] mm - Collision with {obstacle_label}")
            else:
                rejected_count += 1
                print(f"  ✗ Skipped far pose (reach): [{x:.1f}, {y:.1f}, {z:.1f}] mm - Distance {distance:.1f} mm > {MAX_REACH} mm")

    # Add 4 fully extended poses at maximum reach (left, up, right, forward)
    if num_poses_requested > 13:
        print(f"\nGenerating fully extended poses at maximum reach:")
        extended_poses = [
            (-MAX_REACH, 0.0, 0.0, "left"),    # Fully extended to the left
            (0.0, 0.0, MAX_REACH, "up"),       # Fully extended upward
            (MAX_REACH, 0.0, 0.0, "right"),    # Fully extended to the right
            (0.0, MAX_REACH, 0.0, "forward")   # Fully extended forward
        ]
        
        for x, y, z, direction in extended_poses:
            pose_pos = np.array([x, y, z])
            distance = np.linalg.norm(pose_pos - arm_base)
            
            # Default orientation: pointing down (-Z direction)
            o_x = 0.0
            o_y = 0.0
            o_z = -1.0
            theta = 90.0
            
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
            
            print(f"  ✓ Pose {pose_count} ({direction}): [{x:.1f}, {y:.1f}, {z:.1f}] mm (distance: {distance:.1f} mm)")
    
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
