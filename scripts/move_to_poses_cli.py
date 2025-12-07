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


async def main_async():
    parser = argparse.ArgumentParser(
        description='Move arm through calibration poses with speed control',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        '--poses-file',
        nargs='?',
        default='ee_poses_generated.json',
        help='JSON file containing poses (default: ee_poses_generated.json)'
    )
    
    parser.add_argument(
        '--env-file',
        required=True,
        help='Path to .env file with robot configuration'
    )
    
    args = parser.parse_args()
    
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
    part_id = os.getenv('VIAM_PART_ID')
    
    # Validate
    if not all([api_key, api_key_id, robot_address, part_id]):
        print("Error: Missing required configuration!")
        print("Set VIAM_API_KEY, VIAM_API_KEY_ID, VIAM_ROBOT_ADDRESS, VIAM_PART_ID")
        print("Either in .env file or via command line arguments")
        sys.exit(1)
    
    # Load poses
    poses_path = Path(args.poses_file)
    if not poses_path.exists():
        print(f"Error: Poses file '{args.poses_file}' not found!")
        sys.exit(1)
    
    with open(poses_path, 'r') as f:
        poses = json.load(f)
    
    total_poses = len(poses)
    print(f"Found {total_poses} poses in {args.poses_file}")
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
    
    # Move through poses using CLI
    for i, pose_entry in enumerate(poses):
        try:
            pose = pose_entry['data']['pose']
            x, y, z = pose['x'], pose['y'], pose['z']
            ox, oy, oz = pose['o_x'], pose['o_y'], pose['o_z']
            theta = pose['theta']
        except KeyError as e:
            print(f"Error: Missing field in pose {i + 1}: {e}")
            continue
        
        print("=" * 50)
        print(f"Pose {i + 1} of {total_poses}")
        print("=" * 50)
        print(f"Position: ({x:.2f}, {y:.2f}, {z:.2f})")
        print(f"Orientation: ({ox:.4f}, {oy:.4f}, {oz:.4f}) @ {theta:.2f}°")
        print()
        
        # Build CLI command
        cmd = [
            'viam', 'machines', 'part', 'motion', 'set-pose',
            '--part', part_id,
            '--component', component,
            '-x', str(x), '-y', str(y), '-z', str(z),
            '--ox', str(ox), '--oy', str(oy), '--oz', str(oz),
            '--theta', str(theta)
        ]
        
        cmd.insert(0, 'sudo')
        
        print("Moving to pose...")
        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            print("✓ Move completed")
            if result.stdout.strip():
                print(result.stdout)
        except subprocess.CalledProcessError as e:
            print(f"✗ Move failed: {e.stderr}")
            input("Press Enter to continue or Ctrl+C to abort...")
            continue
        
        if i < total_poses - 1:
            print()
            try:
                input("Press Enter for next pose (Ctrl+C to stop)...")
            except KeyboardInterrupt:
                print("\n\nStopped by user.")
                break
            print()
    
    # Restore original speed
    if original_speed:
        print()
        print("Restoring original arm speed...")
        robot = await connect_robot(api_key, api_key_id, robot_address)
        arm = Arm.from_robot(robot, component)
        await set_arm_speed(arm, original_speed)
        print(f"✓ Speed restored to {original_speed}%")
        await robot.close()
    
    print()
    print("=" * 50)
    print("All poses completed!")
    print("=" * 50)


def main():
    asyncio.run(main_async())


if __name__ == '__main__':
    main()
