#!/usr/bin/env python3
"""
Wrapper script for generating and visualizing calibration poses.
Combines generate_poses.py and display_poses_html.py functionality.
"""

import argparse
import subprocess
import sys
from pathlib import Path


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
        "-s", "--safety-height",
        type=float,
        default=50.0,
        help="Safety clearance above mesh surface (mm)"
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
        help="Path to mesh file"
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
        help="JSON file containing obstacle bounding boxes (optional)"
    )
    parser.add_argument(
        "--ee-length",
        type=float,
        default=0.0,
        help="End effector length extending below mounting point in mm (Z dimension)"
    )
    parser.add_argument(
        "--ee-width",
        type=float,
        default=100.0,
        help="End effector width in mm (X dimension)"
    )
    parser.add_argument(
        "--ee-depth",
        type=float,
        default=100.0,
        help="End effector depth in mm (Y dimension)"
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
    
    args = parser.parse_args()
    
    # Get the script directory
    script_dir = Path(__file__).parent
    
    # Step 1: Generate poses
    print("=" * 60)
    print("STEP 1: Generating poses")
    print("=" * 60)
    
    generate_cmd = [
        sys.executable,
        str(script_dir / "generate_poses.py"),
        "--num-poses", str(args.num_poses),
        "--safety-height", str(args.safety_height),
        "--reach", str(args.reach),
        "--arm-base-x", str(args.arm_base_x),
        "--arm-base-y", str(args.arm_base_y),
        "--arm-base-z", str(args.arm_base_z),
        "--mesh", args.mesh,
        "--output", args.output,
        "--ee-length", str(args.ee_length),
        "--ee-width", str(args.ee_width),
        "--ee-depth", str(args.ee_depth),
        "--ee-clearance", str(args.ee_clearance)
    ]
    
    if args.obstacles:
        generate_cmd.extend(["--obstacles", args.obstacles])
    
    result = subprocess.run(generate_cmd)
    if result.returncode != 0:
        print("\n❌ Failed to generate poses", file=sys.stderr)
        return 1
    
    # Step 2: Generate visualization (unless skipped)
    if not args.skip_visualization:
        print("\n" + "=" * 60)
        print("STEP 2: Generating 3D visualization")
        print("=" * 60)
        
        visualize_cmd = [
            sys.executable,
            str(script_dir / "display_poses_html.py"),
            args.output,
            "--reach", str(args.reach),
            "--arm-base-x", str(args.arm_base_x),
            "--arm-base-y", str(args.arm_base_y),
            "--arm-base-z", str(args.arm_base_z),
            "--mesh", args.mesh
        ]
        
        if args.obstacles:
            visualize_cmd.extend(["--obstacles", args.obstacles])
        
        result = subprocess.run(visualize_cmd)
        if result.returncode != 0:
            print("\n⚠️  Failed to generate visualization", file=sys.stderr)
            return 1
        
        # Step 3: Open in browser (if requested)
        if args.open_browser:
            print("\n" + "=" * 60)
            print("STEP 3: Opening visualization in browser")
            print("=" * 60)
            
            html_file = Path.cwd() / "visualization.html"
            if html_file.exists():
                import webbrowser
                webbrowser.open(f"file://{html_file.absolute()}")
                print(f"✓ Opened {html_file}")
            else:
                print(f"⚠️  Could not find {html_file}", file=sys.stderr)
    
    print("\n" + "=" * 60)
    print("✓ Done!")
    print("=" * 60)
    print(f"Poses saved to: {args.output}")
    if not args.skip_visualization:
        print(f"Visualization: visualization.html")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
