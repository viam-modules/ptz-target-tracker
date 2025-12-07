#!/usr/bin/env python3
"""
Calculate hardtop bounding box from mesh file for manual obstacle configuration.

Usage:
    # From mesh file (automatic calculation)
    python scripts/calculate_hardtop_bbox.py hardtop.stl
    
    # From mesh file with custom units
    python scripts/calculate_hardtop_bbox.py hardtop.ply --units meters
    
    # Manual dimensions (if no mesh file)
    python scripts/calculate_hardtop_bbox.py --manual \
        --center-x 0 --center-y 0 --center-z 250 \
        --width 1000 --length 1000 --height 500
"""

import argparse
import sys
from pathlib import Path
import trimesh
import numpy as np


def calculate_bbox_from_mesh(mesh_file: str, units: str = 'mm', num_zones: int = 1):
    """Calculate bounding box(es) from mesh file, optionally splitting into height zones."""
    try:
        mesh = trimesh.load(mesh_file)
        print(f"Loaded mesh: {mesh_file}")
        print(f"  Vertices: {len(mesh.vertices)}")
        print(f"  Faces: {len(mesh.faces)}")
        print()
        
        # Convert based on units
        if units == 'millimeters' or units == 'mm':
            # Mesh is already in mm, no conversion needed
            scale = 1.0
            unit_label = 'millimeters (no conversion)'
        else:
            # Mesh is in meters, convert to mm
            scale = 1000.0
            unit_label = 'meters (converting to mm)'
        
        print(f"Mesh units assumed: {unit_label}")
        print()
        
        vertices = mesh.vertices * scale
        
        # Get overall bounds
        overall_min = np.min(vertices, axis=0)
        overall_max = np.max(vertices, axis=0)
        
        print(f"Overall mesh bounds:")
        print(f"  X: {overall_min[0]:.1f} to {overall_max[0]:.1f} mm")
        print(f"  Y: {overall_min[1]:.1f} to {overall_max[1]:.1f} mm")
        print(f"  Z: {overall_min[2]:.1f} to {overall_max[2]:.1f} mm")
        print()
        
        if num_zones == 1:
            # Single bounding box
            center = (overall_min + overall_max) / 2
            dimensions = overall_max - overall_min
            
            return [(
                center[0], center[1], center[2],
                dimensions[0], dimensions[1], dimensions[2],
                overall_min[0], overall_max[0],
                overall_min[1], overall_max[1],
                overall_min[2], overall_max[2]
            )]
        else:
            # Multiple zones based on Z height
            z_min = overall_min[2]
            z_max = overall_max[2]
            z_range = z_max - z_min
            zone_height = z_range / num_zones
            
            print(f"Splitting into {num_zones} height zones:")
            print(f"  Zone height: {zone_height:.1f} mm")
            print()
            
            zones = []
            for i in range(num_zones):
                zone_z_min = z_min + i * zone_height
                zone_z_max = z_min + (i + 1) * zone_height
                
                # Get vertices in this Z range
                mask = (vertices[:, 2] >= zone_z_min) & (vertices[:, 2] <= zone_z_max)
                zone_vertices = vertices[mask]
                
                if len(zone_vertices) == 0:
                    print(f"  Zone {i+1}: No vertices (skipping)")
                    continue
                
                # Calculate bounds for this zone
                zone_min = np.min(zone_vertices, axis=0)
                zone_max = np.max(zone_vertices, axis=0)
                
                # Use actual Z bounds from vertices, not forced zone bounds
                center = (zone_min + zone_max) / 2
                dimensions = zone_max - zone_min
                
                print(f"  Zone {i+1}:")
                print(f"    Vertices: {len(zone_vertices)}")
                print(f"    Z range: {zone_min[2]:.1f} to {zone_max[2]:.1f} mm")
                print(f"    Dimensions: {dimensions[0]:.1f} x {dimensions[1]:.1f} x {dimensions[2]:.1f} mm")
                
                zones.append((
                    center[0], center[1], center[2],
                    dimensions[0], dimensions[1], dimensions[2],
                    zone_min[0], zone_max[0],
                    zone_min[1], zone_max[1],
                    zone_min[2], zone_max[2]
                ))
            
            print()
            return zones
        
    except Exception as e:
        print(f"Error loading mesh: {e}")
        sys.exit(1)


def calculate_bbox_manual(center_x, center_y, center_z, width, length, height):
    """Calculate bounding box from manual dimensions."""
    x_min = center_x - width / 2
    x_max = center_x + width / 2
    y_min = center_y - length / 2
    y_max = center_y + length / 2
    z_min = center_z - height / 2
    z_max = center_z + height / 2
    
    return (center_x, center_y, center_z, width, length, height,
            x_min, x_max, y_min, y_max, z_min, z_max)


def main():
    parser = argparse.ArgumentParser(
        description='Calculate bounding box for hardtop obstacle from mesh file',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # From mesh file (automatic)
  %(prog)s hardtop.stl
  %(prog)s hardtop.ply --units meters
  
  # Manual dimensions
  %(prog)s --manual --center-x 0 --center-y 0 --center-z 250 --width 1000 --length 1000 --height 500
        '''
    )
    
    parser.add_argument('mesh_file', nargs='?',
                       help='Path to hardtop mesh file (.stl, .ply, .obj, etc.)')
    parser.add_argument('--units', choices=['mm', 'millimeters', 'm', 'meters'],
                       default='meters',
                       help='Units of the mesh file (default: meters)')
    parser.add_argument('--zones', type=int, default=3,
                       help='Number of height zones to split mesh into (default: 3)')
    parser.add_argument('--manual', action='store_true',
                       help='Use manual dimensions instead of mesh file')
    parser.add_argument('--center-x', type=float,
                       help='X coordinate of obstacle center (mm, for manual mode)')
    parser.add_argument('--center-y', type=float,
                       help='Y coordinate of obstacle center (mm, for manual mode)')
    parser.add_argument('--center-z', type=float,
                       help='Z coordinate of obstacle center (mm, for manual mode)')
    parser.add_argument('--width', type=float,
                       help='Width (X dimension) in mm (for manual mode)')
    parser.add_argument('--length', type=float,
                       help='Length (Y dimension) in mm (for manual mode)')
    parser.add_argument('--height', type=float,
                       help='Height (Z dimension) in mm (for manual mode)')
    
    args = parser.parse_args()
    
    # Determine mode
    if args.manual:
        # Manual mode - require all dimensions
        if not all([args.center_x is not None, args.center_y is not None, 
                   args.center_z is not None, args.width, args.length, args.height]):
            print("Error: Manual mode requires --center-x, --center-y, --center-z, --width, --length, --height")
            sys.exit(1)
        
        result = [calculate_bbox_manual(
            args.center_x, args.center_y, args.center_z,
            args.width, args.length, args.height
        )]
    else:
        # Mesh file mode
        if not args.mesh_file:
            print("Error: Provide a mesh file or use --manual mode")
            parser.print_help()
            sys.exit(1)
        
        if not Path(args.mesh_file).exists():
            print(f"Error: Mesh file not found: {args.mesh_file}")
            sys.exit(1)
        
        result = calculate_bbox_from_mesh(args.mesh_file, args.units, args.zones)
    
    # Print results for each zone
    for zone_idx, zone_data in enumerate(result):
        center_x, center_y, center_z, width, length, height, x_min, x_max, y_min, y_max, z_min, z_max = zone_data
        
        zone_label = f"Zone {zone_idx + 1}" if len(result) > 1 else "Single Box"
    
        print()
        print("=" * 70)
        print(f"HARDTOP OBSTACLE BOUNDING BOX - {zone_label}")
        print("=" * 70)
        print()
        print("Input Parameters:")
        print(f"  Center: ({center_x:.1f}, {center_y:.1f}, {center_z:.1f}) mm")
        print(f"  Dimensions: {width:.1f} x {length:.1f} x {height:.1f} mm")
        print()
        print("Bounding Box:")
        print(f"  X range: {x_min:.1f} to {x_max:.1f} mm")
        print(f"  Y range: {y_min:.1f} to {y_max:.1f} mm")
        print(f"  Z range: {z_min:.1f} to {z_max:.1f} mm")
        print()
        print("Corner Coordinates (8 corners):")
        print(f"  1. ({x_min:.1f}, {y_min:.1f}, {z_min:.1f})")
        print(f"  2. ({x_max:.1f}, {y_min:.1f}, {z_min:.1f})")
        print(f"  3. ({x_min:.1f}, {y_max:.1f}, {z_min:.1f})")
        print(f"  4. ({x_max:.1f}, {y_max:.1f}, {z_min:.1f})")
        print(f"  5. ({x_min:.1f}, {y_min:.1f}, {z_max:.1f})")
        print(f"  6. ({x_max:.1f}, {y_min:.1f}, {z_max:.1f})")
        print(f"  7. ({x_min:.1f}, {y_max:.1f}, {z_max:.1f})")
        print(f"  8. ({x_max:.1f}, {y_max:.1f}, {z_max:.1f})")
        print()
        print("=" * 70)
        print(f"VIAM CONFIGURATION - {zone_label}")
        print("=" * 70)
        print()
        print("Add this to your robot configuration obstacles array:")
        print()
        print('{')
        print('  "geometry": {')
        print('    "translation": {')
        print('      "x": 0,')
        print('      "y": 0,')
        print('      "z": 0')
        print('    },')
        print('    "type": "box",')
        print(f'    "x": {width:.1f},')
        print(f'    "y": {length:.1f},')
        print(f'    "z": {height:.1f}')
        print('  },')
        obstacle_label = f"hardtop_zone{zone_idx + 1}" if len(result) > 1 else "hardtop"
        print(f'  "label": "{obstacle_label}",')
        print('  "parent": "world",')
        print('  "translation": {')
        print(f'    "x": {center_x:.1f},')
        print(f'    "y": {center_y:.1f},')
        print(f'    "z": {center_z:.1f}')
        print('  }')
        print('}')
        print()
if __name__ == '__main__':
    main()
