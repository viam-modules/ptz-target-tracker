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
        
        # Add small sphere at position
        sphere = trimesh.creation.icosphere(radius=20)
        sphere.apply_translation(position)
        sphere.visual.vertex_colors = [255, 0, 0, 255]
        scene.add_geometry(sphere, node_name=f'pose_{i}')
        geometry_metadata.append({
            'index': geometry_index,
            'type': 'pose',
            'name': f'pose_{i}'
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
    scene.export('visualization.glb')
    
    print("\n✅ Saved to visualization.glb")
    
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
        let obstacleBoxes = [];
        let meshObjects = [];
        let originSphere = null;
        let reachSphere = null;
        let meshIndex = 0;  // Track which mesh we're processing
        
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
                            child.material = new THREE.MeshStandardMaterial({{
                                color: 0xff0000,
                                emissive: 0xff0000,
                                emissiveIntensity: 1.5,
                                metalness: 0,
                                roughness: 0.2
                            }});
                            child.userData.isPose = true;
                            poseSpheres.push(child);
                            console.log('Added pose sphere to array');
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
            window.addEventListener('mousemove', onMouseMove, false);
            
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
            
            document.getElementById('toggle-origin').addEventListener('change', function(e) {{
                if (originSphere) originSphere.visible = e.target.checked;
            }});
            
            document.getElementById('toggle-reach').addEventListener('change', function(e) {{
                if (reachSphere) reachSphere.visible = e.target.checked;
            }});
            
            document.getElementById('toggle-obstacles').addEventListener('change', function(e) {{
                obstacleBoxes.forEach(obj => obj.visible = e.target.checked);
            }});
        }}
        
        function onMouseMove(event) {{
            if (!scene || !camera || !raycaster) return;
            
            mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
            mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
            
            raycaster.setFromCamera(mouse, camera);
            const intersects = raycaster.intersectObjects(scene.children, true);
            
            let foundObject = false;
            if (intersects.length > 0) {{
                console.log('Intersected', intersects.length, 'objects');
                for (let i = 0; i < intersects.length; i++) {{
                    const obj = intersects[i].object;
                    console.log('Checking object:', obj.userData);
                    
                    // Check for pose spheres
                    if (obj.userData.isPose || obj.userData.isOrigin) {{
                        // Get sphere center from bounding box (more reliable)
                        const box = new THREE.Box3().setFromObject(obj);
                        const sphereCenter = box.getCenter(new THREE.Vector3());
                        
                        const label = obj.userData.isOrigin ? 'Origin' : 'Pose Position';
                        
                        // Calculate distance from origin
                        const distanceFromOrigin = Math.sqrt(
                            sphereCenter.x * sphereCenter.x +
                            sphereCenter.y * sphereCenter.y +
                            sphereCenter.z * sphereCenter.z
                        );
                        
                        console.log('Found sphere:', label, 'at', sphereCenter);
                        
                        tooltip.style.display = 'block';
                        tooltip.style.left = event.clientX + 15 + 'px';
                        tooltip.style.top = event.clientY + 15 + 'px';
                        
                        let tooltipContent = `<strong>${{label}}:</strong><br>
                            X: ${{sphereCenter.x.toFixed(2)}} mm<br>
                            Y: ${{sphereCenter.y.toFixed(2)}} mm<br>
                            Z: ${{sphereCenter.z.toFixed(2)}} mm`;
                        
                        if (obj.userData.isPose) {{
                            tooltipContent += `<br><strong>Distance from origin:</strong> ${{distanceFromOrigin.toFixed(2)}} mm`;
                        }}
                        
                        tooltip.innerHTML = tooltipContent;
                        foundObject = true;
                        break;
                    }}
                    
                    // Check for obstacle boxes
                    if (obj.userData.isObstacle && obj.userData.obstacleData) {{
                        const obstacleData = obj.userData.obstacleData;
                        const label = obstacleData.label || 'Obstacle';
                        const geom = obstacleData.geometry;
                        const trans = obstacleData.translation;
                        
                        tooltip.style.display = 'block';
                        tooltip.style.left = event.clientX + 15 + 'px';
                        tooltip.style.top = event.clientY + 15 + 'px';
                        
                        tooltip.innerHTML = `<strong>${{label}}:</strong><br>
                            Center: (${{trans.x.toFixed(1)}}, ${{trans.y.toFixed(1)}}, ${{trans.z.toFixed(1)}}) mm<br>
                            Size: ${{geom.x.toFixed(1)}} x ${{geom.y.toFixed(1)}} x ${{geom.z.toFixed(1)}} mm`;
                        
                        foundObject = true;
                        break;
                    }}
                }}
            }}
            
            if (!foundObject) {{
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
    
    with open('visualization.html', 'w') as f:
        f.write(html_content)
    
    print("✅ Saved to visualization.html")
    print("\nOpen http://localhost:8000/visualization.html in your browser")
    print("(Make sure web server is running: python3 -m http.server 8000)")

if __name__ == "__main__":
    main()
