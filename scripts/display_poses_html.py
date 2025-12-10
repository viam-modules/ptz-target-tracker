import json
import trimesh
import numpy as np
import argparse
import sys
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description='Generate 3D visualization of poses')
    parser.add_argument(
        'poses_file',
        nargs='?',
        default='ee_poses_generated.json',
        help='JSON file containing poses (default: ee_poses_generated.json)'
    )
    parser.add_argument(
        '--reach',
        type=float,
        default=1700.0,
        help='Arm reach in mm for visualization sphere (default: 1700.0)'
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
        '--mesh',
        type=str,
        default='mesh.ply',
        help='Path to mesh file (default: mesh.ply)'
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
        help='End effector size in X dimension in mm (default: 100)'
    )
    parser.add_argument(
        '--ee-y',
        type=float,
        default=100.0,
        help='End effector size in Y dimension in mm (default: 100)'
    )
    parser.add_argument(
        '--ee-z',
        type=float,
        default=0.0,
        help='End effector size in Z dimension (extending below mounting point) in mm (default: 0)'
    )
    parser.add_argument(
        '--results',
        type=str,
        help='JSON file containing pose execution results (visited/failed poses)'
    )
    
    args = parser.parse_args()
    
    # Check if poses file exists
    if not Path(args.poses_file).exists():
        print(f"Error: Poses file '{args.poses_file}' not found!")
        sys.exit(1)
    
    arm_base = np.array([args.arm_base_x, args.arm_base_y, args.arm_base_z])
    
    # Load mesh
    try:
        mesh = trimesh.load(args.mesh)
        print(f"Loaded mesh with {len(mesh.vertices)} vertices")
        
        # Convert mesh from meters to millimeters
        mesh.apply_scale(1000)
        print(f"Mesh bounds (mm): {mesh.bounds}")
        
        # Color the mesh - bright cyan/turquoise for visibility
        mesh.visual.vertex_colors = [0, 200, 255, 255]
    except Exception as e:
        print(f"Warning: Could not load mesh.ply: {e}")
        mesh = None
    
    # Load poses
    with open(args.poses_file, 'r') as f:
        poses_data = json.load(f)
    
    print(f"Loaded {len(poses_data)} poses from {args.poses_file}")
    
    # Load results if provided
    visited_indices = set()
    failed_indices = set()
    if args.results and Path(args.results).exists():
        with open(args.results, 'r') as f:
            results_data = json.load(f)
        visited_indices = {item['index'] for item in results_data.get('visited', [])}
        failed_indices = {item['index'] for item in results_data.get('failed', [])}
        print(f"Loaded results: {len(visited_indices)} visited, {len(failed_indices)} failed")
    
    # Create scene
    scene = trimesh.Scene()
    
    # Track geometry metadata for JavaScript
    geometry_metadata = []
    geometry_index = 0
    
    if mesh is not None:
        scene.add_geometry(mesh, node_name=f'mesh_main')
        geometry_metadata.append({
            'index': geometry_index,
            'type': 'mesh',
            'name': 'mesh_main'
        })
        geometry_index += 1
    
    # Add poses
    positions = []
    for i, pose_entry in enumerate(poses_data):
        pose = pose_entry['data']['pose']
        position = [pose['x'], pose['y'], pose['z']]
        positions.append(position)
        
        # Determine color based on results
        if i in visited_indices:
            color = [0, 255, 0, 255]  # Green for visited
        elif i in failed_indices:
            color = [255, 0, 0, 255]  # Red for failed
        else:
            color = [255, 255, 0, 255]  # Yellow for untested
        
        # Add small sphere at position
        sphere = trimesh.creation.icosphere(radius=20)
        sphere.apply_translation(position)
        sphere.visual.vertex_colors = color
        scene.add_geometry(sphere, node_name=f'pose_{i}')
        geometry_metadata.append({
            'index': geometry_index,
            'type': 'pose',
            'name': f'pose_{i}',
            'pose_number': i,
            'visited': i in visited_indices,
            'failed': i in failed_indices
        })
        geometry_index += 1
        
        # Add end effector box at this pose
        if args.ee_x > 0 or args.ee_y > 0 or args.ee_z > 0:
            ee_size = [args.ee_x, args.ee_y, args.ee_z if args.ee_z > 0 else 10]
            ee_box = trimesh.creation.box(extents=ee_size)
            # Position the box: center at pose point - [0, 0, z/2]
            # This makes the top of the box at the pose point, extending downward in Z
            ee_center = [position[0], position[1], position[2] - ee_size[2]/2]
            ee_box.apply_translation(ee_center)
            ee_box.visual.vertex_colors = [255, 255, 0, 150]  # Yellow, semi-transparent
            scene.add_geometry(ee_box, node_name=f'end_effector_{i}')
            geometry_metadata.append({
                'index': geometry_index,
                'type': 'end_effector',
                'name': f'end_effector_{i}',
                'pose_index': i
            })
            geometry_index += 1
    
    # Add sphere at arm base position
    base_sphere = trimesh.creation.icosphere(radius=40)  # Slightly larger
    base_sphere.apply_translation(arm_base)
    base_sphere.visual.vertex_colors = [144, 238, 144, 255]  # Light green
    scene.add_geometry(base_sphere, node_name='origin')
    geometry_metadata.append({
        'index': geometry_index,
        'type': 'origin',
        'name': 'origin'
    })
    geometry_index += 1
    print(f"Added arm base marker at ({arm_base[0]:.1f}, {arm_base[1]:.1f}, {arm_base[2]:.1f}) in light green")
    
    # Add translucent sphere showing arm reach
    reach_sphere = trimesh.creation.icosphere(subdivisions=4, radius=args.reach)
    reach_sphere.apply_translation(arm_base)
    reach_sphere.visual.vertex_colors = [100, 150, 255, 40]  # Light blue, semi-transparent
    scene.add_geometry(reach_sphere, node_name='reach_sphere')
    geometry_metadata.append({
        'index': geometry_index,
        'type': 'reach',
        'name': 'reach_sphere'
    })
    geometry_index += 1
    print(f"Added arm reach sphere at base (radius: {args.reach:.0f}mm, translucent)")
    
    # Add obstacle bounding boxes if provided
    obstacles_data = []
    if args.obstacles and Path(args.obstacles).exists():
        with open(args.obstacles, 'r') as f:    
            obstacles_data = json.load(f)
        print(f"\nLoaded {len(obstacles_data)} obstacles from {args.obstacles}")
        
        for i, obstacle in enumerate(obstacles_data):
            label = obstacle.get('label', 'unknown')
            geometry = obstacle['geometry']
            translation = obstacle['translation']
            
            # Create a box at the center with the given dimensions
            box_size = [geometry['x'], geometry['y'], geometry['z']]
            box_center = [translation['x'], translation['y'], translation['z']]
            
            # Create wireframe box
            box = trimesh.creation.box(extents=box_size)
            box.apply_translation(box_center)
            # Make it semi-transparent orange/yellow
            box.visual.vertex_colors = [255, 165, 0, 100]  # Orange, semi-transparent
            scene.add_geometry(box, node_name=f'obstacle_{i}')
            geometry_metadata.append({
                'index': geometry_index,
                'type': 'obstacle',
                'name': f'obstacle_{i}',
                'label': label
            })
            geometry_index += 1
            
            print(f"  Added obstacle '{label}': center=({translation['x']:.1f}, {translation['y']:.1f}, {translation['z']:.1f}), "
                  f"size=({geometry['x']:.1f} x {geometry['y']:.1f} x {geometry['z']:.1f})")
    
    positions = np.array(positions)
    unique_positions = np.unique(positions, axis=0)
    
    print(f"\nUnique positions: {len(unique_positions)}")
    print(f"Position range - X: [{positions[:, 0].min():.1f}, {positions[:, 0].max():.1f}]")
    print(f"Position range - Y: [{positions[:, 1].min():.1f}, {positions[:, 1].max():.1f}]")
    print(f"Position range - Z: [{positions[:, 2].min():.1f}, {positions[:, 2].max():.1f}]")
    
    # Export to GLB (3D model format that browsers/viewers can open)
    # Save in the same directory as the poses file
    poses_dir = Path(args.poses_file).parent
    output_glb = poses_dir / 'visualization.glb'
    scene.export(str(output_glb))
    
    print(f"\n✅ Saved to {output_glb}")
    
    # Generate HTML viewer
    # Embed obstacles data and geometry metadata as JSON in the HTML
    obstacles_json = json.dumps(obstacles_data)
    metadata_json = json.dumps(geometry_metadata)
    
    # Add timestamp to force cache refresh
    import time
    timestamp = int(time.time())
    
    html_content = f"""<!DOCTYPE html>
<html>
<head>
    <title>PTZ Target Tracker Visualization</title>
    <!-- Generated: {timestamp} -->
    <style>
        body {{ 
            margin: 0; 
            background: white;
            overflow: hidden;
        }}
        canvas {{ 
            display: block; 
        }}
        #info {{
            position: absolute;
            top: 10px;
            left: 10px;
            color: #333;
            font-family: Arial, sans-serif;
            background: rgba(255, 255, 255, 0.9);
            padding: 10px;
            border-radius: 5px;
        }}
        #controls {{
            position: absolute;
            top: 10px;
            right: 10px;
            color: #333;
            font-family: Arial, sans-serif;
            background: rgba(255, 255, 255, 0.9);
            padding: 15px;
            border-radius: 5px;
            min-width: 180px;
        }}
        #controls h3 {{
            margin: 0 0 10px 0;
            font-size: 16px;
            border-bottom: 1px solid #ccc;
            padding-bottom: 5px;
        }}
        #controls label {{
            display: block;
            margin: 8px 0;
            cursor: pointer;
            user-select: none;
        }}
        #controls input[type="checkbox"] {{
            margin-right: 8px;
            cursor: pointer;
        }}
        #tooltip {{
            position: absolute;
            padding: 8px 12px;
            background: rgba(0, 0, 0, 0.8);
            color: white;
            border-radius: 4px;
            font-family: Arial, sans-serif;
            font-size: 14px;
            pointer-events: none;
            display: none;
            z-index: 1000;
        }}
        #legend {{
            position: absolute;
            bottom: 20px;
            left: 20px;
            background: rgba(255, 255, 255, 0.9);
            padding: 15px;
            border-radius: 5px;
            font-family: Arial, sans-serif;
            font-size: 14px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.2);
        }}
        #legend h3 {{
            margin: 0 0 10px 0;
            font-size: 16px;
            border-bottom: 1px solid #ccc;
            padding-bottom: 5px;
        }}
        #legend .axis-item {{
            display: flex;
            align-items: center;
            margin: 8px 0;
        }}
        #legend .axis-color {{
            width: 30px;
            height: 3px;
            margin-right: 10px;
            border-radius: 2px;
        }}
        #legend .x-axis {{
            background: #ff0000;
        }}
        #legend .y-axis {{
            background: #00ff00;
        }}
        #legend .z-axis {{
            background: #0000ff;
        }}
    </style>
</head>
<body>
    <div id="info">
        <strong>Controls:</strong><br>
        Left click + drag: Rotate<br>
        Right click + drag: Pan<br>
        Scroll: Zoom
    </div>
    <div id="controls">
        <h3>Visibility</h3>
        <label>
            <input type="checkbox" id="toggle-mesh" checked>
            Mesh
        </label>
        <label>
            <input type="checkbox" id="toggle-poses" checked>
            Pose Spheres
        </label>
        <label>
            <input type="checkbox" id="toggle-end-effectors" checked>
            End Effectors
        </label>
        <label>
            <input type="checkbox" id="toggle-origin" checked>
            Arm Base
        </label>
        <label>
            <input type="checkbox" id="toggle-reach" checked>
            Reach Sphere
        </label>
        <label>
            <input type="checkbox" id="toggle-obstacles" checked>
            Obstacles
        </label>
        <h3>Pose Filter</h3>
        <label>
            <input type="checkbox" id="filter-visited" checked>
            Visited (Green)
        </label>
        <label>
            <input type="checkbox" id="filter-failed" checked>
            Failed (Red)
        </label>
        <label>
            <input type="checkbox" id="filter-untested" checked>
            Untested (Yellow)
        </label>
    </div>
    <div id="legend">
        <h3>Axes</h3>
        <div class="axis-item">
            <div class="axis-color x-axis"></div>
            <span>X Axis (Red)</span>
        </div>
        <div class="axis-item">
            <div class="axis-color y-axis"></div>
            <span>Y Axis (Green)</span>
        </div>
        <div class="axis-item">
            <div class="axis-color z-axis"></div>
            <span>Z Axis (Blue)</span>
        </div>
    </div>
    <div id="tooltip"></div>
    <script type="importmap">
        {{
            "imports": {{
                "three": "https://unpkg.com/three@0.158.0/build/three.module.js",
                "three/addons/": "https://unpkg.com/three@0.158.0/examples/jsm/"
            }}
        }}
    </script>
    <script type="module">
        import * as THREE from 'three';
        import {{ OrbitControls }} from 'three/addons/controls/OrbitControls.js';
        import {{ GLTFLoader }} from 'three/addons/loaders/GLTFLoader.js';

        const obstaclesData = {obstacles_json};
        const geometryMetadata = {metadata_json};
        
        let scene, camera, renderer, controls;
        let raycaster, mouse, tooltip;
        let poseSpheres = [];
        let endEffectors = [];
        let obstacleBoxes = [];
        let meshObjects = [];
        let originSphere = null;
        let reachSphere = null;
        let meshIndex = 0;  // Track which mesh we're processing
        let highlightedObject = null;
        let originalMaterial = null;
        
        function init() {{
            console.log('Initializing scene...');
            
            raycaster = new THREE.Raycaster();
            mouse = new THREE.Vector2();
            tooltip = document.getElementById('tooltip');
            
            scene = new THREE.Scene();
            scene.background = new THREE.Color(0xffffff);
            
            camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 100000);
            camera.position.set(3000, 3000, 3000);
            
            renderer = new THREE.WebGLRenderer({{ antialias: true }});
            renderer.setSize(window.innerWidth, window.innerHeight);
            document.body.appendChild(renderer.domElement);
            
            controls = new OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;
            
            const ambientLight = new THREE.AmbientLight(0xffffff, 1.5);
            scene.add(ambientLight);
            
            // Add axes helper (XYZ reference frame)
            const axesHelper = new THREE.AxesHelper(500);  // 500mm axes length
            scene.add(axesHelper);
            
            const light1 = new THREE.DirectionalLight(0xffffff, 0.8);
            light1.position.set(1000, 1000, 1000);
            scene.add(light1);
            
            const light2 = new THREE.DirectionalLight(0xffffff, 0.5);
            light2.position.set(-1000, -1000, -1000);
            scene.add(light2);
            
            const light3 = new THREE.DirectionalLight(0xffffff, 0.5);
            light3.position.set(1000, -1000, 1000);
            scene.add(light3);
            
            const loader = new GLTFLoader();
            loader.load('visualization.glb', function(gltf) {{
                gltf.scene.traverse(function(child) {{
                    if (child.isMesh) {{
                        child.material.side = THREE.DoubleSide;
                        
                        // Get metadata for this mesh by index
                        const metadata = geometryMetadata[meshIndex];
                        const meshType = metadata ? metadata.type : 'unknown';
                        const meshName = metadata ? metadata.name : 'unknown';
                        
                        console.log('Mesh found - index:', meshIndex, 'type:', meshType, 'name:', meshName);
                        
                        if (meshType === 'pose') {{
                            // Color will be set based on visited/failed status from trimesh
                            // Green (0x00ff00) for visited, Red (0xff0000) for failed/untested
                            const isVisited = metadata.visited || false;
                            const isFailed = metadata.failed || false;
                            const baseColor = isVisited ? 0x00ff00 : 0xff0000;
                            
                            child.material = new THREE.MeshStandardMaterial({{
                                color: baseColor,
                                emissive: baseColor,
                                emissiveIntensity: 1.5,
                                metalness: 0,
                                roughness: 0.2
                            }});
                            child.userData.isPose = true;
                            child.userData.poseNumber = metadata.pose_number;
                            child.userData.visited = isVisited;
                            child.userData.failed = isFailed;
                            poseSpheres.push(child);
                            console.log('Added pose sphere to array - visited:', isVisited, 'failed:', isFailed);
                        }} else if (meshType === 'end_effector') {{
                            child.material = new THREE.MeshStandardMaterial({{
                                color: 0xFFFF00,
                                emissive: 0x888800,
                                emissiveIntensity: 0.2,
                                transparent: true,
                                opacity: 0.6,
                                side: THREE.DoubleSide,
                                metalness: 0.1,
                                roughness: 0.8
                            }});
                            child.userData.isEndEffector = true;
                            child.userData.poseIndex = metadata.pose_index;
                            endEffectors.push(child);
                            console.log('Added end effector box to array');
                        }} else if (meshType === 'origin') {{
                            child.material = new THREE.MeshStandardMaterial({{
                                color: 0x90EE90,
                                emissive: 0x90EE90,
                                emissiveIntensity: 1.5,
                                metalness: 0,
                                roughness: 0.2
                            }});
                            child.userData.isOrigin = true;
                            originSphere = child;
                            console.log('Added ORIGIN sphere to array');
                        }} else if (meshType === 'reach') {{
                            child.material = new THREE.MeshStandardMaterial({{
                                color: 0x6496FF,
                                transparent: true,
                                opacity: 0.05,
                                side: THREE.DoubleSide,
                                metalness: 0,
                                roughness: 1.0,
                                depthWrite: false,
                                blending: THREE.NormalBlending
                            }});
                            child.renderOrder = -1;  // Render behind other objects
                            reachSphere = child;
                            console.log('Added REACH sphere (highly translucent)');
                        }} else if (meshType === 'obstacle') {{
                            // Replace solid mesh with wireframe for obstacles
                            const edges = new THREE.EdgesGeometry(child.geometry);
                            const lineMaterial = new THREE.LineBasicMaterial({{ color: 0xFF6600, linewidth: 2 }});
                            const wireframe = new THREE.LineSegments(edges, lineMaterial);
                            wireframe.position.copy(child.position);
                            wireframe.rotation.copy(child.rotation);
                            wireframe.scale.copy(child.scale);
                            
                            // Store obstacle data from metadata
                            const obstacleIndex = obstacleBoxes.length;
                            wireframe.userData.isObstacle = true;
                            wireframe.userData.obstacleIndex = obstacleIndex;
                            wireframe.userData.obstacleLabel = metadata.label;
                            if (obstacleIndex < obstaclesData.length) {{
                                wireframe.userData.obstacleData = obstaclesData[obstacleIndex];
                            }}
                            
                            scene.add(wireframe);
                            obstacleBoxes.push(wireframe);
                            
                            // Also add the semi-transparent fill and store it with the wireframe
                            child.material = new THREE.MeshStandardMaterial({{
                                color: 0xFFA500,
                                transparent: true,
                                opacity: 0.1,
                                side: THREE.DoubleSide,
                                depthWrite: false
                            }});
                            child.userData.isObstacleFill = true;
                            child.userData.obstacleIndex = obstacleIndex;
                            if (obstacleIndex < obstaclesData.length) {{
                                child.userData.obstacleData = obstaclesData[obstacleIndex];
                            }}
                            obstacleBoxes.push(child);  // Add the fill mesh to the obstacles array too
                            console.log('Added OBSTACLE box wireframe and fill');
                        }} else if (meshType === 'mesh') {{
                            // This is the main mesh - make it cyan/turquoise and visible
                            child.material = new THREE.MeshStandardMaterial({{
                                color: 0x00C8FF,
                                emissive: 0x006080,
                                emissiveIntensity: 0.3,
                                metalness: 0.1,
                                roughness: 0.7,
                                side: THREE.DoubleSide
                            }});
                            child.userData.isMesh = true;
                            meshObjects.push(child);
                            console.log('Added MESH with cyan color');
                        }} else {{
                            console.log('Unknown mesh type:', meshType);
                        }}
                        
                        child.material.needsUpdate = true;
                        meshIndex++;  // Increment for next mesh
                    }}
                }});
                
                console.log('Loaded', meshIndex, 'meshes total');
                console.log('Pose spheres:', poseSpheres.length);
                console.log('End effectors:', endEffectors.length);
                console.log('Origin sphere:', originSphere ? 'yes' : 'no');
                console.log('Reach sphere:', reachSphere ? 'yes' : 'no');
                console.log('Obstacles:', obstacleBoxes.length);
                console.log('Mesh objects:', meshObjects.length);
                
                scene.add(gltf.scene);
                
                const box = new THREE.Box3().setFromObject(gltf.scene);
                const center = box.getCenter(new THREE.Vector3());
                
                controls.target.copy(center);
                controls.update();
            }});
            
            window.addEventListener('resize', onWindowResize, false);
            window.addEventListener('click', onClick, false);
            
            // Close tooltip on ESC key
            window.addEventListener('keydown', function(e) {{
                if (e.key === 'Escape') {{
                    clearHighlight();
                    tooltip.style.display = 'none';
                }}
            }});
            
            // Setup visibility toggle controls
            setupVisibilityControls();
            
            animate();
        }}
        
        function setupVisibilityControls() {{
            document.getElementById('toggle-mesh').addEventListener('change', function(e) {{
                meshObjects.forEach(obj => obj.visible = e.target.checked);
            }});
            
            document.getElementById('toggle-poses').addEventListener('change', function(e) {{
                poseSpheres.forEach(obj => obj.visible = e.target.checked);
            }});
            
            document.getElementById('toggle-end-effectors').addEventListener('change', function(e) {{
                endEffectors.forEach(obj => obj.visible = e.target.checked);
            }});
            
            document.getElementById('toggle-origin').addEventListener('change', function(e) {{
                if (originSphere) originSphere.visible = e.target.checked;
            }});
            
            document.getElementById('toggle-reach').addEventListener('change', function(e) {{
                if (reachSphere) reachSphere.visible = e.target.checked;
            }});
            
            document.getElementById('toggle-obstacles').addEventListener('change', function(e) {{
                obstacleBoxes.forEach(obj => obj.visible = e.target.checked);
            }});
            
            // Pose filter controls
            const updatePoseVisibility = () => {{
                const showVisited = document.getElementById('filter-visited').checked;
                const showFailed = document.getElementById('filter-failed').checked;
                const showUntested = document.getElementById('filter-untested').checked;
                const showPoses = document.getElementById('toggle-poses').checked;
                const showEndEffectors = document.getElementById('toggle-end-effectors').checked;
                
                poseSpheres.forEach((sphere) => {{
                    let shouldShow = showPoses;
                    if (sphere.userData.visited) {{
                        shouldShow = shouldShow && showVisited;
                    }} else if (sphere.userData.failed) {{
                        shouldShow = shouldShow && showFailed;
                    }} else {{
                        shouldShow = shouldShow && showUntested;
                    }}
                    sphere.visible = shouldShow;
                }});
                
                // Update end effectors based on their associated pose visibility
                endEffectors.forEach((ee) => {{
                    const poseIdx = ee.userData.poseIndex;
                    if (poseIdx !== undefined && poseIdx < poseSpheres.length) {{
                        const sphere = poseSpheres[poseIdx];
                        ee.visible = sphere.visible && showEndEffectors;
                    }}
                }});
            }};
            
            document.getElementById('filter-visited').addEventListener('change', updatePoseVisibility);
            document.getElementById('filter-failed').addEventListener('change', updatePoseVisibility);
            document.getElementById('filter-untested').addEventListener('change', updatePoseVisibility);
        }}
        
        function clearHighlight() {{
            if (highlightedObject && originalMaterial) {{
                highlightedObject.material = originalMaterial;
                highlightedObject = null;
                originalMaterial = null;
            }}
        }}
        
        function highlightObject(obj) {{
            clearHighlight();
            highlightedObject = obj;
            originalMaterial = obj.material.clone();
            
            // Create highlighted material
            const highlightMaterial = obj.material.clone();
            
            if (obj.userData.isPose || obj.userData.isOrigin) {{
                // Make poses brighter and add emissive
                highlightMaterial.emissive = new THREE.Color(0xFFFF00);
                highlightMaterial.emissiveIntensity = 2.0;
            }} else if (obj.userData.isEndEffector) {{
                highlightMaterial.color = new THREE.Color(0xFFFF00);
                highlightMaterial.emissive = new THREE.Color(0xFFAA00);
                highlightMaterial.emissiveIntensity = 0.8;
                highlightMaterial.opacity = 0.9;
            }} else if (obj.userData.isMesh) {{
                highlightMaterial.color = new THREE.Color(0x00FFFF);
                highlightMaterial.emissive = new THREE.Color(0x00AAFF);
                highlightMaterial.emissiveIntensity = 0.7;
            }} else {{
                // Generic highlight - brighten the material
                const currentColor = new THREE.Color(highlightMaterial.color);
                currentColor.multiplyScalar(1.5);
                highlightMaterial.color = currentColor;
                highlightMaterial.emissive = currentColor;
                highlightMaterial.emissiveIntensity = 0.5;
            }}
            
            obj.material = highlightMaterial;
        }}
        
        function onClick(event) {{
            if (!scene || !camera || !raycaster) return;
            
            mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
            mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
            
            raycaster.setFromCamera(mouse, camera);
            const intersects = raycaster.intersectObjects(scene.children, true);
            
            let foundObject = false;
            if (intersects.length > 0) {{
                for (let i = 0; i < intersects.length; i++) {{
                    const obj = intersects[i].object;
                    
                    // Skip invisible objects
                    if (!obj.visible) continue;
                    
                    // Check for pose spheres
                    if (obj.userData.isPose || obj.userData.isOrigin) {{
                        // Skip invisible objects
                        if (!obj.visible) continue;
                        
                        highlightObject(obj);
                        
                        const box = new THREE.Box3().setFromObject(obj);
                        const sphereCenter = box.getCenter(new THREE.Vector3());
                        
                        const label = obj.userData.isOrigin ? 'Arm Base (Origin)' : 'Calibration Pose';
                        const icon = obj.userData.isOrigin ? '&#127919;' : '&#128205;';  // 🎯 and 📍
                        
                        const distanceFromOrigin = Math.sqrt(
                            sphereCenter.x * sphereCenter.x +
                            sphereCenter.y * sphereCenter.y +
                            sphereCenter.z * sphereCenter.z
                        );
                        
                        tooltip.style.display = 'block';
                        tooltip.style.left = event.clientX + 15 + 'px';
                        tooltip.style.top = event.clientY + 15 + 'px';
                        
                        let tooltipContent = `<strong>${{icon}} ${{label}}</strong><br>
                            <strong>Position:</strong><br>
                            &nbsp;&nbsp;X: ${{sphereCenter.x.toFixed(1)}} mm<br>
                            &nbsp;&nbsp;Y: ${{sphereCenter.y.toFixed(1)}} mm<br>
                            &nbsp;&nbsp;Z: ${{sphereCenter.z.toFixed(1)}} mm`;
                        
                        if (obj.userData.isPose) {{
                            tooltipContent += `<br><strong>Pose #:</strong> ${{obj.userData.poseNumber + 1}}`;
                            tooltipContent += `<br><strong>Distance:</strong> ${{distanceFromOrigin.toFixed(1)}} mm`;
                            
                            if (obj.userData.visited) {{
                                tooltipContent += `<br><strong>Status:</strong> &#9989; Visited`;  // ✅
                            }} else if (obj.userData.failed) {{
                                tooltipContent += `<br><strong>Status:</strong> &#10060; Failed`;  // ❌
                            }} else {{
                                tooltipContent += `<br><strong>Status:</strong> &#9898; Not tested`;  // ⚪
                            }}
                        }}
                        
                        tooltip.innerHTML = tooltipContent;
                        foundObject = true;
                        break;
                    }}
                    
                    // Check for end effectors
                    if (obj.userData.isEndEffector) {{
                        highlightObject(obj);
                        
                        const box = new THREE.Box3().setFromObject(obj);
                        const center = box.getCenter(new THREE.Vector3());
                        const size = new THREE.Vector3();
                        box.getSize(size);
                        
                        tooltip.style.display = 'block';
                        tooltip.style.left = event.clientX + 15 + 'px';
                        tooltip.style.top = event.clientY + 15 + 'px';
                        
                        tooltip.innerHTML = `<strong>&#128295; End Effector</strong><br>
                            <strong>Center:</strong><br>
                            &nbsp;&nbsp;X: ${{center.x.toFixed(1)}} mm<br>
                            &nbsp;&nbsp;Y: ${{center.y.toFixed(1)}} mm<br>
                            &nbsp;&nbsp;Z: ${{center.z.toFixed(1)}} mm<br>
                            <strong>Size:</strong> ${{size.x.toFixed(0)}} &times; ${{size.y.toFixed(0)}} &times; ${{size.z.toFixed(0)}} mm`;
                        
                        foundObject = true;
                        break;
                    }}
                    
                    // Check for reach sphere
                    if (obj === reachSphere) {{
                        // Don't highlight reach sphere as it's translucent
                        
                        const box = new THREE.Box3().setFromObject(obj);
                        const radius = box.max.x - box.min.x;  // Diameter / 2
                        
                        tooltip.style.display = 'block';
                        tooltip.style.left = event.clientX + 15 + 'px';
                        tooltip.style.top = event.clientY + 15 + 'px';
                        
                        tooltip.innerHTML = `<strong>&#128309; Arm Reach Sphere</strong><br>
                            <strong>Radius:</strong> ${{(radius / 2).toFixed(0)}} mm<br>
                            <strong>Center:</strong> Origin (0, 0, 0)`;
                        
                        foundObject = true;
                        break;
                    }}
                    
                    // Check for obstacle boxes
                    if ((obj.userData.isObstacle || obj.userData.isObstacleFill) && obj.userData.obstacleData) {{
                        // Highlight only the fill, not wireframe
                        if (obj.userData.isObstacleFill) {{
                            highlightObject(obj);
                        }}
                        
                        const obstacleData = obj.userData.obstacleData;
                        const label = obstacleData.label || 'Obstacle';
                        const geom = obstacleData.geometry;
                        const trans = obstacleData.translation;
                        
                        tooltip.style.display = 'block';
                        tooltip.style.left = event.clientX + 15 + 'px';
                        tooltip.style.top = event.clientY + 15 + 'px';
                        
                        tooltip.innerHTML = `<strong>&#128679; ${{label}}</strong><br>
                            <strong>Center:</strong><br>
                            &nbsp;&nbsp;X: ${{trans.x.toFixed(1)}} mm<br>
                            &nbsp;&nbsp;Y: ${{trans.y.toFixed(1)}} mm<br>
                            &nbsp;&nbsp;Z: ${{trans.z.toFixed(1)}} mm<br>
                            <strong>Dimensions:</strong> ${{geom.x.toFixed(0)}} &times; ${{geom.y.toFixed(0)}} &times; ${{geom.z.toFixed(0)}} mm`;
                        
                        foundObject = true;
                        break;
                    }}
                    
                    // Check for main mesh
                    if (obj.userData.isMesh) {{
                        highlightObject(obj);
                        
                        const box = new THREE.Box3().setFromObject(obj);
                        const min = box.min;
                        const max = box.max;
                        const size = new THREE.Vector3();
                        box.getSize(size);
                        
                        tooltip.style.display = 'block';
                        tooltip.style.left = event.clientX + 15 + 'px';
                        tooltip.style.top = event.clientY + 15 + 'px';
                        
                        tooltip.innerHTML = `<strong>&#127959; Environment Mesh</strong><br>
                            <strong>Bounds:</strong><br>
                            &nbsp;&nbsp;X: [${{min.x.toFixed(0)}}, ${{max.x.toFixed(0)}}] mm<br>
                            &nbsp;&nbsp;Y: [${{min.y.toFixed(0)}}, ${{max.y.toFixed(0)}}] mm<br>
                            &nbsp;&nbsp;Z: [${{min.z.toFixed(0)}}, ${{max.z.toFixed(0)}}] mm<br>
                            <strong>Size:</strong> ${{size.x.toFixed(0)}} &times; ${{size.y.toFixed(0)}} &times; ${{size.z.toFixed(0)}} mm`;
                        
                        foundObject = true;
                        break;
                    }}
                }}
            }}
            
            if (!foundObject) {{
                clearHighlight();
                tooltip.style.display = 'none';
            }}
        }}
        
        function onWindowResize() {{
            camera.aspect = window.innerWidth / window.innerHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight);
        }}
        
        function animate() {{
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        }}
        
        init();
    </script>
</body>
</html>"""
    
    # Save in the same directory as the poses file
    output_html = poses_dir / 'visualization.html'
    with open(output_html, 'w') as f:
        f.write(html_content)
    
    print(f"✅ Saved to {output_html}")
    print(f"\nOpen http://localhost:8000/{output_html.name} in your browser")
    print("(Make sure web server is running: python3 -m http.server 8000)")

if __name__ == "__main__":
    main()
