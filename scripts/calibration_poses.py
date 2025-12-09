#!/usr/bin/env python3
"""
Wrapper script for generating and visualizing calibration poses.
Combines generate_poses.py and display_poses_html.py functionality.
"""

import argparse
import subprocess
import sys
from pathlib import Path
from datetime import datetime
import shutil
import trimesh
import numpy as np
import json
import os


def main():
    parser = argparse.ArgumentParser(
        description="Generate calibration poses and visualize them in 3D",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # Pose generation parameters
    parser.add_argument(
        "-n", "--num-poses",
        type=int,
        default=24,
        help="Number of poses to generate (12-24 recommended)"
    )
    parser.add_argument(
        "-r", "--reach",
        type=float,
        default=1700.0,
        help="Maximum reach of the arm (mm). UR5e: ~850, UR20: ~1700"
    )
    
    # Arm base position
    parser.add_argument(
        "--arm-base-x",
        type=float,
        default=0.0,
        help="X coordinate of arm base (mm)"
    )
    parser.add_argument(
        "--arm-base-y",
        type=float,
        default=0.0,
        help="Y coordinate of arm base (mm)"
    )
    parser.add_argument(
        "--arm-base-z",
        type=float,
        default=0.0,
        help="Z coordinate of arm base (mm)"
    )
    
    # Input/Output options
    parser.add_argument(
        "-m", "--mesh",
        type=str,
        default="mesh.ply",
        help="Path to mesh file (not required if --dry-run is used)"
    )
    parser.add_argument(
        "--dry-run",
        type=str,
        help="Dry run mode: provide bounding box as 'x_min,y_min,z_min,x_max,y_max,z_max' in mm to generate a simple box mesh for testing"
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        default="ee_poses_generated.json",
        help="Output JSON file for poses"
    )
    parser.add_argument(
        "--obstacles",
        type=str,
        help="JSON file containing obstacle bounding boxes (optional). If not provided, will be auto-generated from mesh"
    )
    parser.add_argument(
        "--obstacle-zones",
        type=int,
        default=3,
        help="Number of height zones to split obstacles into (default: 3)"
    )
    parser.add_argument(
        "--skip-obstacles",
        action="store_true",
        help="Skip obstacle generation entirely"
    )
    parser.add_argument(
        "--ee-x",
        type=float,
        default=100.0,
        help="End effector size in X dimension in mm (default: 100)"
    )
    parser.add_argument(
        "--ee-y",
        type=float,
        default=100.0,
        help="End effector size in Y dimension in mm (default: 100)"
    )
    parser.add_argument(
        "--ee-z",
        type=float,
        default=0.0,
        help="End effector size in Z dimension (extending below mounting point) in mm (default: 0)"
    )
    parser.add_argument(
        "--ee-clearance",
        type=float,
        default=50.0,
        help="Additional safety clearance around end effector in mm"
    )
    parser.add_argument(
        "--skip-visualization",
        action="store_true",
        help="Skip generating the 3D visualization"
    )
    parser.add_argument(
        "--open-browser",
        action="store_true",
        help="Open the visualization in browser after generation"
    )
    parser.add_argument(
        "--results",
        type=str,
        help="JSON file containing pose execution results (visited/failed poses) for visualization"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        help="Output directory for all artifacts. If not provided, creates timestamped directory (calibration_YYYYMMDD_HHMMSS)"
    )
    
    args = parser.parse_args()
    
    # Create output directory with timestamp
    if args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path(f"ptz_tracking_calibration_{timestamp}")
    
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"📁 Output directory: {output_dir.absolute()}\n")
    
    # Handle dry run mode - generate mesh from bounding box
    if args.dry_run:
        try:
            bbox_values = [float(x) for x in args.dry_run.split(',')]
            if len(bbox_values) != 6:
                raise ValueError("Expected 6 values")
            x_min, y_min, z_min, x_max, y_max, z_max = bbox_values
            
            # Create box mesh from bounding box
            center = np.array([
                (x_min + x_max) / 2,
                (y_min + y_max) / 2,
                (z_min + z_max) / 2
            ])
            extents = np.array([
                x_max - x_min,
                y_max - y_min,
                z_max - z_min
            ])
            
            # Create box mesh (trimesh uses meters, convert from mm)
            box_mesh = trimesh.creation.box(extents / 1000.0)
            box_mesh.apply_translation(center / 1000.0)
            
            # Save to output directory
            mesh_path = output_dir / "dry_run_mesh.ply"
            box_mesh.export(str(mesh_path))
            
            # Update args.mesh to use the generated mesh
            args.mesh = str(mesh_path)
            
            print(f"🧪 DRY RUN MODE")
            print(f"   Generated mesh from bounding box:")
            print(f"   X: [{x_min}, {x_max}] mm")
            print(f"   Y: [{y_min}, {y_max}] mm")
            print(f"   Z: [{z_min}, {z_max}] mm")
            print(f"   Saved to: {mesh_path}\n")
            
        except ValueError as e:
            print(f"❌ Error: Invalid --dry-run format. Expected 'x_min,y_min,z_min,x_max,y_max,z_max' in mm")
            print(f"   Example: --dry-run '-2000,500,-600,2000,2200,-300'")
            sys.exit(1)
    else:
        # Use provided mesh file
        mesh_path = Path(args.mesh)
        if not mesh_path.exists():
            print(f"❌ Error: Mesh file not found: {mesh_path}")
            print(f"   Tip: Use --dry-run 'x_min,y_min,z_min,x_max,y_max,z_max' for testing without a mesh file")
            sys.exit(1)
    
    # Get the script directory
    script_dir = Path(__file__).parent
    
    # Define output files in the output directory
    obstacles_file_path = output_dir / "obstacles.json"
    poses_file_path = output_dir / args.output
    visualization_html_path = output_dir / "visualization.html"
    visualization_glb_path = output_dir / "visualization.glb"
    
    # Step 0: Generate obstacles from mesh (if needed)
    obstacles_file = args.obstacles
    if not args.skip_obstacles and not obstacles_file:
        print("=" * 60)
        print("STEP 0: Generating obstacles from mesh")
        print("=" * 60)
        
        # Generate obstacles.json in output directory
        obstacle_cmd = [
            sys.executable,
            str(script_dir / "calculate_hardtop_bbox.py"),
            args.mesh,
            "--zones", str(args.obstacle_zones),
            "-o", str(obstacles_file_path)
        ]
        
        result = subprocess.run(obstacle_cmd)
        if result.returncode != 0:
            print("\n⚠️  Failed to generate obstacles, continuing without obstacles...", file=sys.stderr)
            obstacles_file = None
        else:
            print(f"✅ Generated obstacles: {obstacles_file_path}")
            obstacles_file = str(obstacles_file_path)
    
    # Step 1: Generate poses
    print("\n" + "=" * 60)
    print("STEP 1: Generating poses")
    print("=" * 60)
    
    generate_cmd = [
        sys.executable,
        str(script_dir / "generate_poses.py"),
        "--num-poses", str(args.num_poses),
        "--reach", str(args.reach),
        "--arm-base-x", str(args.arm_base_x),
        "--arm-base-y", str(args.arm_base_y),
        "--arm-base-z", str(args.arm_base_z),
        "--mesh", args.mesh,
        "--output", str(poses_file_path),
        "--ee-x", str(args.ee_x),
        "--ee-y", str(args.ee_y),
        "--ee-z", str(args.ee_z),
        "--ee-clearance", str(args.ee_clearance)
    ]
    
    if obstacles_file:
        generate_cmd.extend(["--obstacles", obstacles_file])
    
    result = subprocess.run(generate_cmd)
    if result.returncode != 0:
        print("\n❌ Failed to generate poses", file=sys.stderr)
        return 1
    
    # Save visualization metadata for later use
    viz_metadata = {
        "reach": args.reach,
        "arm_base_x": args.arm_base_x,
        "arm_base_y": args.arm_base_y,
        "arm_base_z": args.arm_base_z,
        "ee_x": args.ee_x,
        "ee_y": args.ee_y,
        "ee_z": args.ee_z,
        "mesh_file": os.path.basename(args.mesh) if not args.dry_run else "dry_run_mesh.ply"
    }
    viz_metadata_file = output_dir / "visualization_metadata.json"
    with open(viz_metadata_file, 'w') as f:
        json.dump(viz_metadata, f, indent=2)
    
    # Step 2: Generate visualization (unless skipped)
    if not args.skip_visualization:
        print("\n" + "=" * 60)
        print("STEP 2: Generating 3D visualization")
        print("=" * 60)
        
        visualize_cmd = [
            sys.executable,
            str(script_dir / "display_poses_html.py"),
            str(poses_file_path),
            "--reach", str(args.reach),
            "--arm-base-x", str(args.arm_base_x),
            "--arm-base-y", str(args.arm_base_y),
            "--arm-base-z", str(args.arm_base_z),
            "--mesh", args.mesh,
            "--ee-x", str(args.ee_x),
            "--ee-y", str(args.ee_y),
            "--ee-z", str(args.ee_z)
        ]
        
        if obstacles_file:
            visualize_cmd.extend(["--obstacles", obstacles_file])
        
        if args.results:
            visualize_cmd.extend(["--results", args.results])
        
        result = subprocess.run(visualize_cmd)
        if result.returncode != 0:
            print("\n⚠️  Failed to generate visualization", file=sys.stderr)
            return 1
        
        # Move generated visualization files to output directory
        if Path("visualization.html").exists():
            shutil.move("visualization.html", visualization_html_path)
        if Path("visualization.glb").exists():
            shutil.move("visualization.glb", visualization_glb_path)
        
        # Step 3: Open in browser (if requested)
        if args.open_browser:
            print("\n" + "=" * 60)
            print("STEP 3: Opening visualization in browser")
            print("=" * 60)
            
            if visualization_html_path.exists():
                import webbrowser
                webbrowser.open(f"file://{visualization_html_path.absolute()}")
                print(f"✓ Opened {visualization_html_path}")
            else:
                print(f"⚠️  Could not find {visualization_html_path}", file=sys.stderr)
    
    # Copy mesh file to output directory for reference (unless dry-run, where it's already there)
    mesh_path = Path(args.mesh)
    if mesh_path.exists() and not args.dry_run:
        shutil.copy(mesh_path, output_dir / mesh_path.name)
    
    print("\n" + "=" * 60)
    print("✓ Done!")
    print("=" * 60)
    print(f"📁 All artifacts saved to: {output_dir.absolute()}")
    print(f"  - Poses: {poses_file_path.name}")
    if obstacles_file and Path(obstacles_file).exists():
        print(f"  - Obstacles: {obstacles_file_path.name}")
    if not args.skip_visualization:
        print(f"  - Visualization: {visualization_html_path.name}")
        print(f"  - 3D Model: {visualization_glb_path.name}")
    print(f"  - Mesh (copy): {mesh_path.name}")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
