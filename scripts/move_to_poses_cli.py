#!/usr/bin/env python3
"""
Script to move arm through calibration poses.
Sets arm speed at the beginning, uses Viam CLI for moves, restores speed at the end.

Configuration:
    Create a .env file in the project root with:
        VIAM_API_KEY=your-api-key
        VIAM_API_KEY_ID=your-api-key-id
        VIAM_ROBOT_ADDRESS=your-robot-address
        VIAM_COMPONENT_NAME=your-component-name
        VIAM_PART_ID=your-part-id
    
Usage: 
    python move_to_poses_cli.py [poses_file] [--speed-percent 25]
"""

import json
import sys
import argparse
import os
import asyncio
import subprocess
from pathlib import Path
from dotenv import load_dotenv

from viam.robot.client import RobotClient
from viam.components.arm import Arm
from viam.services.generic import Generic as GenericService

        
        
DEFAULT_VELOCITY_NORMAL = 10
DEFAULT_VELOCITY_SLOW = 10

async def connect_robot(api_key: str, api_key_id: str, robot_address: str):
    """Connect to the robot."""
    opts = RobotClient.Options.with_api_key(
        api_key=api_key,
        api_key_id=api_key_id
    )
    return await RobotClient.at_address(robot_address, opts)


async def set_arm_speed(arm: Arm, speed_percent: float):
    """Set arm speed via DoCommand."""
    try:
        await arm.do_command({"set_vel": speed_percent})
        return True
    except Exception as e:
        print(f"Error setting arm speed: {e}")
        return False


async def get_arm_speed(arm: Arm):
    """Get current arm speed setting."""
    try:
        result = await arm.do_command({"get_speed_pct": True})
        if "speed_pct" in result:
            return result["speed_pct"]
    except Exception:
        pass
    return None


async def push_sample(component):
    """Record current calibration sample via DoCommand."""
    try:
        result = await component.do_command({"command": "push-sample"})
        return True, result
    except Exception as e:
        return False, str(e)


async def pop_sample(component):
    """Delete the last calibration sample via DoCommand."""
    try:
        result = await component.do_command({"command": "pop-sample"})
        return True, result
    except Exception as e:
        return False, str(e)


async def get_calibration_samples(component):
    """Get all calibration samples via DoCommand."""
    try:
        result = await component.do_command({"command": "get-calibration-samples"})
        return True, result
    except Exception as e:
        return False, str(e)


def move_to_pose(pose_entry, part_id: str, component: str):
    """
    Move to a specific pose using Viam CLI.
    
    Args:
        pose_entry: Pose data dictionary
        part_id: Viam part ID
        component: Component name
        
    Returns:
        tuple: (success: bool, stdout: str, stderr: str)
    """
    try:
        pose = pose_entry['data']['pose']
        x, y, z = pose['x'], pose['y'], pose['z']
        ox, oy, oz = pose['o_x'], pose['o_y'], pose['o_z']
        theta = pose['theta']
    except KeyError as e:
        return False, "", f"Missing field in pose: {e}"
    
    # Build CLI command
    cmd = [
        'sudo', 'viam', 'machines', 'part', 'motion', 'set-pose',
        '--part', part_id,
        '--component', component,
        '-x', str(x), '-y', str(y), '-z', str(z),
        '--ox', str(ox), '--oy', str(oy), '--oz', str(oz),
        '--theta', str(theta)
    ]
    
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        return True, result.stdout, ""
    except subprocess.CalledProcessError as e:
        return False, "", str(e.stderr)


async def main_async():
    parser = argparse.ArgumentParser(
        description='Move arm through calibration poses with speed control',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        'calibration_dir',
        help='Directory containing calibration files (poses, obstacles, visualization, etc.)'
    )
    
    parser.add_argument(
        '--env-file',
        required=True,
        help='Path to .env file with robot configuration'
    )
    
    parser.add_argument(
        '--record-samples',
        action='store_true',
        help='Record calibration sample at each pose using push_sample do_command'
    )
    
    args = parser.parse_args()
    
    # Validate calibration directory
    calib_dir = Path(args.calibration_dir)
    if not calib_dir.exists():
        print(f"Error: Calibration directory not found: {args.calibration_dir}")
        sys.exit(1)
    
    if not calib_dir.is_dir():
        print(f"Error: Path is not a directory: {args.calibration_dir}")
        sys.exit(1)
    
    # Set up logging to file and terminal
    log_file = calib_dir / "move_to_poses_log.txt"
    
    class TeeLogger:
        """Write to both file and terminal"""
        def __init__(self, filename):
            self.terminal = sys.stdout
            self.log = open(filename, 'w', encoding='utf-8', buffering=1)  # Line buffering
        
        def write(self, message):
            self.terminal.write(message)
            self.log.write(message)
            self.log.flush()
        
        def flush(self):
            self.terminal.flush()
            self.log.flush()
        
        def close(self):
            self.log.close()
    
    # Redirect stdout to both terminal and log file
    tee = TeeLogger(log_file)
    old_stdout = sys.stdout
    sys.stdout = tee
    
    print(f"📝 Logging to: {log_file.absolute()}\n")
    
    # Find poses file in directory
    poses_file = calib_dir / "ee_poses_generated.json"
    if not poses_file.exists():
        print(f"Error: Poses file not found: {poses_file}")
        sys.exit(1)
    
    print(f"Using calibration directory: {calib_dir}")
    print(f"Poses file: {poses_file}")
    
    # Load .env file (required)
    env_path = Path(args.env_file)
    if not env_path.exists():
        print(f"Error: .env file not found: {args.env_file}")
        sys.exit(1)
    
    load_dotenv(env_path)
    print(f"Loaded configuration from: {args.env_file}")
    print()
    
    # Get values with priority: CLI args > .env > defaults
    api_key = os.getenv('VIAM_API_KEY')
    api_key_id = os.getenv('VIAM_API_KEY_ID')
    robot_address = os.getenv('VIAM_ROBOT_ADDRESS')
    component = os.getenv('VIAM_COMPONENT_NAME', 'ptz_fake_arm_2')
    tracker_component = os.getenv('VIAM_TRACKER_COMPONENT_NAME', 'ptz-component-tracker')
    part_id = os.getenv('VIAM_PART_ID')
    
    # Validate
    if not all([api_key, api_key_id, robot_address, part_id]):
        print("Error: Missing required configuration!")
        print("Set VIAM_API_KEY, VIAM_API_KEY_ID, VIAM_ROBOT_ADDRESS, VIAM_PART_ID")
        print("Either in .env file or via command line arguments")
        sys.exit(1)
    
    # Load poses from calibration directory
    with open(poses_file, 'r') as f:
        poses = json.load(f)
    
    total_poses = len(poses)
    print(f"Found {total_poses} poses in {poses_file}")
    print(f"Component: {component}")
    print(f"Target speed: {DEFAULT_VELOCITY_NORMAL}%")
    print()
    
    # Connect to robot
    print("Connecting to robot...")
    robot = await connect_robot(api_key, api_key_id, robot_address)
    arm = Arm.from_robot(robot, component)
    print("✓ Connected")
    
    # Get original speed
    original_speed = await get_arm_speed(arm)
    if original_speed:
        print(f"Current arm speed: {original_speed}%")
    
    # Set slow speed
    print(f"Setting arm speed to {DEFAULT_VELOCITY_NORMAL}%...")
    if await set_arm_speed(arm, DEFAULT_VELOCITY_NORMAL):
        print("✓ Speed set")
    else:
        print("⚠ Arm does not support speed control via SDK")
        print("  You may need to configure speed limits in your robot config")
        print("  or manually set speed on the arm controller")
        print()
        response = input("Continue anyway? [y/N]: ")
        if response.lower() != 'y':
            print("Aborted.")
            await robot.close()
            sys.exit(0)
    
    # Close connection (CLI will handle moves)
    await robot.close()
    print()
    
    input("Press Enter to start moving through poses...")
    print()
    
    # Track results
    visited_poses = []  # List of (index, pose_entry) tuples
    failed_poses = []   # List of (index, pose_entry, error) tuples
    
    # Move through poses using CLI with ability to go back
    i = 0
    while i < total_poses:
        pose_entry = poses[i]
        
        try:
            pose = pose_entry['data']['pose']
            x, y, z = pose['x'], pose['y'], pose['z']
            ox, oy, oz = pose['o_x'], pose['o_y'], pose['o_z']
            theta = pose['theta']
        except KeyError as e:
            print(f"Error: Missing field in pose {i + 1}: {e}")
            i += 1
            continue
        
        print("=" * 50)
        print(f"Pose {i + 1} of {total_poses}")
        print("=" * 50)
        print(f"Position: ({x:.2f}, {y:.2f}, {z:.2f})")
        print(f"Orientation: ({ox:.4f}, {oy:.4f}, {oz:.4f}) @ {theta:.2f}°")
        print()
        
        print("Moving to pose...")
        success, stdout, stderr = move_to_pose(pose_entry, part_id, component)
        
        if success:
            print("✓ Move completed")
            # Update visited list - remove if already there, add at current position
            visited_poses = [(idx, p) for idx, p in visited_poses if idx != i]
            visited_poses.append((i, pose_entry))
            if stdout.strip():
                print(stdout)
        else:
            print(f"✗ Move failed: {stderr}")
            # Update failed list - remove if already there, add at current position
            failed_poses = [(idx, p, e) for idx, p, e in failed_poses if idx != i]
            failed_poses.append((i, pose_entry, stderr))
        
        if i < total_poses - 1 or i > 0:
            print()
            try:
                # Interactive loop for this pose
                while True:
                    if args.record_samples and success:
                        user_input = input("Press Enter for next pose, 'p' for previous, 's' to save sample, 'd' to delete last sample, or Ctrl+C to stop: ").strip().lower()
                    else:
                        user_input = input("Press Enter for next pose, 'p' for previous pose, or Ctrl+C to stop: ").strip().lower()
                    
                    if user_input == 's' and args.record_samples and success:
                        print("\nRecording calibration sample...")
                        # Need to reconnect to use SDK
                        robot_temp = await connect_robot(api_key, api_key_id, robot_address)
                        try:
                            # Get the tracker service as GenericService
                            tracker = GenericService.from_robot(robot_temp, tracker_component)
                            
                            sample_success, sample_result = await push_sample(tracker)
                            if sample_success:
                                print("✓ Sample recorded")
                                if sample_result:
                                    print(f"  Result: {sample_result}")
                                print()
                                # Successfully saved, break the loop to continue
                                break
                            else:
                                print(f"✗ Failed to record sample: {sample_result}")
                                print()
                                # Stay in loop to allow retry
                        except Exception as e:
                            print(f"✗ Failed to access tracker component '{tracker_component}': {e}")
                            print()
                            # Stay in loop to allow retry
                        finally:
                            await robot_temp.close()
                    
                    elif user_input == 'd' and args.record_samples:
                        print("\nDeleting last calibration sample...")
                        # Need to reconnect to use SDK
                        robot_temp = await connect_robot(api_key, api_key_id, robot_address)
                        try:
                            # Get the tracker service as GenericService
                            tracker = GenericService.from_robot(robot_temp, tracker_component)
                            
                            delete_success, delete_result = await pop_sample(tracker)
                            if delete_success:
                                print("✓ Last sample deleted")
                                if delete_result:
                                    print(f"  Result: {delete_result}")
                            else:
                                print(f"✗ Failed to delete sample: {delete_result}")
                        except Exception as e:
                            print(f"✗ Failed to access tracker component '{tracker_component}': {e}")
                        finally:
                            await robot_temp.close()
                        print()
                        # Stay in loop to ask again
                    
                    else:
                        # Not 's' or 'd', break the loop
                        break
                
                if user_input == 'p' and i > 0:
                    i -= 1
                    print(f"\n↩ Going back to pose {i + 1}")
                    print()
                else:
                    i += 1
                    print()
            except KeyboardInterrupt:
                print("\n\nStopped by user.")
                break
        else:
            i += 1
    
    # Restore original speed and get calibration samples
    print()
    print("Restoring original arm speed...")
    robot = await connect_robot(api_key, api_key_id, robot_address)
    arm = Arm.from_robot(robot, component)
    
    if original_speed:
        await set_arm_speed(arm, original_speed)
        print(f"✓ Speed restored to {original_speed}%")
    
    # Get calibration samples if recording was enabled
    if args.record_samples:
        print("\nRetrieving calibration samples...")
        try:
            # Get the tracker service as GenericService
            tracker = GenericService.from_robot(robot, tracker_component)
            
            samples_success, samples_result = await get_calibration_samples(tracker)
            if samples_success:
                print("✓ Calibration samples retrieved")
                print(f"\nCalibration data:")
                print(json.dumps(samples_result, indent=2))
                
                # Save samples to file
                samples_file = calib_dir / "ee_poses_generated_calibration_samples.json"
                with open(samples_file, 'w') as f:
                    json.dump(samples_result, f, indent=2)
                print(f"\n✓ Calibration samples saved to: {samples_file}")
            else:
                print(f"✗ Failed to retrieve calibration samples: {samples_result}")
        except Exception as e:
            print(f"✗ Failed to access tracker component '{tracker_component}': {e}")
    
    await robot.close()
    
    print()
    print("=" * 50)
    print("All poses completed!")
    print("=" * 50)
    print()
    
    # Print summary
    print("=" * 50)
    print("CALIBRATION SUMMARY")
    print("=" * 50)
    print(f"Total poses: {total_poses}")
    print(f"✓ Successful: {len(visited_poses)} ({len(visited_poses)*100.0/total_poses:.1f}%)")
    print(f"✗ Failed: {len(failed_poses)} ({len(failed_poses)*100.0/total_poses:.1f}%)")
    print()
    
    if visited_poses:
        print(f"Visited poses ({len(visited_poses)}):")
        for idx, pose_entry in visited_poses:
            pose = pose_entry['data']['pose']
            print(f"  ✓ Pose {idx + 1}: ({pose['x']:.2f}, {pose['y']:.2f}, {pose['z']:.2f})")
        print()
    
    if failed_poses:
        print(f"Failed poses ({len(failed_poses)}):")
        for idx, pose_entry, error in failed_poses:
            pose = pose_entry['data']['pose']
            print(f"  ✗ Pose {idx + 1}: ({pose['x']:.2f}, {pose['y']:.2f}, {pose['z']:.2f})")
            # Extract the key error message
            if "physically unreachable" in error:
                print(f"     Reason: Position unreachable (IK solver failed)")
            else:
                print(f"     Reason: {error.split('desc = ')[-1][:80] if 'desc = ' in error else error[:80]}")
        print()
    
    # Save results to JSON for visualization
    results_file = calib_dir / "ee_poses_generated_results.json"
    results_data = {
        'total': total_poses,
        'visited': [{'index': idx, 'pose': pose_entry} for idx, pose_entry in visited_poses],
        'failed': [{'index': idx, 'pose': pose_entry, 'error': error} for idx, pose_entry, error in failed_poses]
    }
    with open(results_file, 'w') as f:
        json.dump(results_data, f, indent=2)
    print(f"Results saved to: {results_file}")
    
    # Regenerate visualization with results
    print("\nRegenerating visualization with execution results...")
    visualization_html = calib_dir / "visualization.html"
    
    if visualization_html.exists():
        # Find the corresponding files
        obstacles_file = calib_dir / "obstacles.json"
        mesh_file = None
        
        # Look for mesh file (try common names)
        for mesh_name in ['dry_run_mesh.ply', 'mesh.ply', calib_dir.name + '.ply']:
            candidate = calib_dir / mesh_name
            if candidate.exists():
                mesh_file = candidate
                break
        
        if mesh_file:
            # Build command to regenerate visualization
            regen_cmd = [
                'python3', 
                str(Path(__file__).parent / 'display_poses_html.py'),
                str(poses_file),
                '--mesh', str(mesh_file),
                '--results', str(results_file)
            ]
            
            if obstacles_file.exists():
                regen_cmd.extend(['--obstacles', str(obstacles_file)])
            
            # Load visualization metadata if available to get original parameters
            viz_metadata_file = calib_dir / "visualization_metadata.json"
            if viz_metadata_file.exists():
                try:
                    with open(viz_metadata_file, 'r') as f:
                        viz_metadata = json.load(f)
                    
                    # Add parameters from metadata
                    if 'reach' in viz_metadata:
                        regen_cmd.extend(['--reach', str(viz_metadata['reach'])])
                    if 'ee_x' in viz_metadata:
                        regen_cmd.extend(['--ee-x', str(viz_metadata['ee_x'])])
                    if 'ee_y' in viz_metadata:
                        regen_cmd.extend(['--ee-y', str(viz_metadata['ee_y'])])
                    if 'ee_z' in viz_metadata:
                        regen_cmd.extend(['--ee-z', str(viz_metadata['ee_z'])])
                    if 'arm_base_x' in viz_metadata:
                        regen_cmd.extend(['--arm-base-x', str(viz_metadata['arm_base_x'])])
                    if 'arm_base_y' in viz_metadata:
                        regen_cmd.extend(['--arm-base-y', str(viz_metadata['arm_base_y'])])
                    if 'arm_base_z' in viz_metadata:
                        regen_cmd.extend(['--arm-base-z', str(viz_metadata['arm_base_z'])])
                    
                    print(f"✓ Loaded visualization parameters from metadata")
                except Exception as e:
                    print(f"⚠ Could not load visualization metadata: {e}")
            
            # Try to extract parameters from existing files
            # Read the meta.json or use defaults
            try:
                result = subprocess.run(regen_cmd, check=True, capture_output=True, text=True)
                print(f"✓ Visualization updated: {visualization_html}")
            except subprocess.CalledProcessError as e:
                print(f"⚠ Failed to regenerate visualization: {e.stderr}")
        else:
            print("⚠ Could not find mesh file to regenerate visualization")
    else:
        print(f"⚠ Visualization file not found: {visualization_html}")
    
    print("=" * 50)
    print(f"📝 Session log saved to: {log_file}")
    
    # Close log file and restore stdout
    sys.stdout = old_stdout
    tee.close()


def main():
    asyncio.run(main_async())


if __name__ == '__main__':
    main()
