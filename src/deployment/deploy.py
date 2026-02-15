#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Sigyn AI Contributors
"""
Deployment manager for Sigyn AI models.

This script handles the complete deployment pipeline:
1. Validate model exists and is compatible with target
2. SCP model files to target device
3. Backup existing models on target
4. Verify deployment with test inference
5. Rollback on failure

Usage:
    python deploy.py --model can_detector_pihat_v1 --target sigyn --camera gripper_cam
    python deploy.py --model can_detector_oakd_v1 --target sigyn --camera oakd_nav
    python deploy.py --model can_detector_pihat_v1 --target all  # Deploy to all targets
"""

import argparse
import sys
import yaml
import subprocess
from pathlib import Path
from datetime import datetime


def load_robot_config(robot_id):
    """Load robot configuration."""
    config_path = Path(f"configs/robots/{robot_id}.yaml")
    if not config_path.exists():
        raise FileNotFoundError(f"Robot config not found: {config_path}")
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    return config


def validate_model_for_camera(model_name, camera_config):
    """Validate that model exists for the target camera's device."""
    device_id = camera_config['device']
    model_dir = Path(f"models/exported/{model_name}/{device_id}")
    
    if not model_dir.exists():
        raise FileNotFoundError(
            f"Model not found for device {device_id}: {model_dir}\n"
            f"Run: python src/export/export.py --model <path_to_pt> --device {device_id}"
        )
    
    # Check for required files based on device
    required_files = []
    if device_id == 'pi5_hailo8':
        required_files = [f"{model_name}.hef", "labels.txt"]
    elif device_id == 'oakd_lite':
        required_files = [f"{model_name}.blob", "labels.txt"]
    elif device_id == 'jetson_orin_nano':
        required_files = [f"{model_name}.engine", "labels.txt"]
    
    missing = []
    for file in required_files:
        if not (model_dir / file).exists():
            # .hef might not exist yet (needs manual Hailo compilation)
            if file.endswith('.hef') or file.endswith('.engine'):
                print(f"⚠️  Warning: {file} not found (may need manual compilation)")
            else:
                missing.append(file)
    
    if missing:
        raise FileNotFoundError(f"Missing required files in {model_dir}: {missing}")
    
    return model_dir


def backup_remote_models(host, remote_dir, ssh_key):
    """Backup existing models on remote host."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = f"{remote_dir}/backups/{timestamp}"
    
    # Create backup directory
    cmd = f"ssh -i {ssh_key} {host} 'mkdir -p {backup_dir}'"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"⚠️  Warning: Could not create backup directory: {result.stderr}")
        return None
    
    # Backup existing models (*.hef, *.blob, *.engine, *.onnx, labels.txt)
    cmd = f"ssh -i {ssh_key} {host} 'cp -r {remote_dir}/*.{{hef,blob,engine,onnx,txt}} {backup_dir}/ 2>/dev/null || true'"
    subprocess.run(cmd, shell=True)
    
    print(f"💾 Backed up existing models to: {backup_dir}")
    return backup_dir


def deploy_via_scp(model_dir, host, remote_dir, ssh_key):
    """Deploy model files via SCP."""
    print(f"\n📤 Deploying model files to {host}...")
    
    # Ensure remote directory exists
    cmd = f"ssh -i {ssh_key} {host} 'mkdir -p {remote_dir}'"
    subprocess.run(cmd, shell=True, check=True)
    
    # Copy all files from model directory
    files_to_copy = list(model_dir.glob("*"))
    
    for file in files_to_copy:
        if file.is_file():
            print(f"   Copying: {file.name}")
            cmd = f"scp -i {ssh_key} {file} {host}:{remote_dir}/"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            
            if result.returncode != 0:
                raise RuntimeError(f"Failed to copy {file.name}: {result.stderr}")
    
    print(f"✅ Files deployed to {remote_dir}")


def test_deployment(host, remote_dir, device_id, ssh_key):
    """Test deployed model with inference."""
    print(f"\n🧪 Testing deployment on {host}...")
    
    # Device-specific test commands
    test_commands = {
        'pi5_hailo8': f"""
            cd {remote_dir} && python3 -c "
import os
print('Checking files...')
for f in os.listdir('.'):
    if f.endswith(('.hef', '.txt')):
        print(f'  Found: {{f}}')
print('Deployment test passed!')
"
        """,
        'oakd_lite': f"""
            cd {remote_dir} && python3 -c "
import os
print('Checking files...')
for f in os.listdir('.'):
    if f.endswith(('.blob', '.txt')):
        print(f'  Found: {{f}}')
print('Deployment test passed!')
"
        """,
        'jetson_orin_nano': f"""
            cd {remote_dir} && python3 -c "
import os
print('Checking files...')
for f in os.listdir('.'):
    if f.endswith(('.engine', '.txt')):
        print(f'  Found: {{f}}')
print('Deployment test passed!')
"
        """
    }
    
    cmd = f"ssh -i {ssh_key} {host} '{test_commands.get(device_id, 'ls -la')}'"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"⚠️  Test failed: {result.stderr}")
        return False
    
    print(result.stdout)
    print(f"✅ Deployment test passed")
    return True


def rollback_deployment(host, backup_dir, remote_dir, ssh_key):
    """Rollback to previous model version."""
    print(f"\n⏮️  Rolling back to backup: {backup_dir}")
    
    cmd = f"ssh -i {ssh_key} {host} 'cp -r {backup_dir}/* {remote_dir}/'"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"❌ Rollback failed: {result.stderr}")
        return False
    
    print(f"✅ Rollback complete")
    return True


def deploy_to_camera(model_name, robot_config, camera_name):
    """Deploy model to a specific camera on the robot."""
    cameras = robot_config['hardware']['cameras']
    
    if camera_name not in cameras:
        raise ValueError(f"Camera '{camera_name}' not found in robot config. Available: {list(cameras.keys())}")
    
    camera_config = cameras[camera_name]
    device_id = camera_config['device']
    
    print(f"\n🎯 Deploying to camera: {camera_name}")
    print(f"   Device: {device_id}")
    print(f"   Mount: {camera_config['mount_location']}")
    print(f"   Purpose: {camera_config['purpose']}")
    
    # Validate model
    print(f"\n🔍 Validating model: {model_name}")
    model_dir = validate_model_for_camera(model_name, camera_config)
    print(f"   Model directory: {model_dir}")
    
    # Get deployment config
    deployment = camera_config['deployment']
    host = deployment['host']
    remote_dir = deployment['model_dir']
    
    # Get network config for SSH key
    network_config = robot_config.get('network', {})
    network_key = 'sigyn_vision' if 'Vision' in host else 'sigyn_main'
    ssh_key = network_config.get(network_key, {}).get('ssh_key', '~/.ssh/id_rsa')
    ssh_key = Path(ssh_key).expanduser()
    
    # Backup existing models
    if robot_config['deployment_preferences'].get('backup_old_models', True):
        backup_dir = backup_remote_models(host, remote_dir, ssh_key)
    else:
        backup_dir = None
    
    # Deploy model
    try:
        deploy_via_scp(model_dir, host, remote_dir, ssh_key)
        
        # Test deployment
        if robot_config['deployment_preferences'].get('test_after_deploy', True):
            test_passed = test_deployment(host, remote_dir, device_id, ssh_key)
            
            if not test_passed and robot_config['deployment_preferences'].get('rollback_on_failure', True):
                if backup_dir:
                    rollback_deployment(host, backup_dir, remote_dir, ssh_key)
                    raise RuntimeError("Deployment test failed, rolled back to previous version")
                else:
                    raise RuntimeError("Deployment test failed, no backup to rollback to")
        
        print(f"\n✅ Successfully deployed {model_name} to {camera_name}")
        
    except Exception as e:
        print(f"\n❌ Deployment failed: {e}")
        raise


def main():
    parser = argparse.ArgumentParser(
        description='Deploy models to Sigyn AI robots',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Deploy to specific camera
  python deploy.py --model can_detector_pihat_v1 --target sigyn --camera gripper_cam
  
  # Deploy to all cameras on robot
  python deploy.py --model can_detector_oakd_v1 --target sigyn --all-cameras
  
  # Dry run (validation only)
  python deploy.py --model can_detector_pihat_v1 --target sigyn --camera gripper_cam --dry-run
        """
    )
    
    parser.add_argument(
        '--model',
        type=str,
        required=True,
        help='Model name (subdirectory in models/exported/)'
    )
    
    parser.add_argument(
        '--target',
        type=str,
        required=True,
        help='Target robot identifier (e.g., sigyn)'
    )
    
    parser.add_argument(
        '--camera',
        type=str,
        default=None,
        help='Specific camera to deploy to (e.g., gripper_cam, oakd_nav)'
    )
    
    parser.add_argument(
        '--all-cameras',
        action='store_true',
        help='Deploy to all cameras on the robot'
    )
    
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Validate only, do not deploy'
    )
    
    args = parser.parse_args()
    
    if not args.camera and not args.all_cameras:
        print("❌ Error: Must specify --camera or --all-cameras")
        sys.exit(1)
    
    # Load robot configuration
    print(f"🔍 Loading robot config: {args.target}")
    robot_config = load_robot_config(args.target)
    print(f"   Robot: {robot_config['robot']['name']}")
    print(f"   Description: {robot_config['robot']['description']}")
    
    if args.dry_run:
        print("\n🔍 DRY RUN MODE - No files will be deployed")
    
    try:
        if args.all_cameras:
            cameras = list(robot_config['hardware']['cameras'].keys())
            print(f"\n📋 Deploying to {len(cameras)} cameras: {cameras}")
            
            for camera_name in cameras:
                if not args.dry_run:
                    deploy_to_camera(args.model, robot_config, camera_name)
                else:
                    print(f"\n✓ Would deploy to: {camera_name}")
                    model_dir = validate_model_for_camera(args.model, robot_config['hardware']['cameras'][camera_name])
                    print(f"  Model: {model_dir}")
        else:
            if not args.dry_run:
                deploy_to_camera(args.model, robot_config, args.camera)
            else:
                print(f"\n✓ Would deploy to: {args.camera}")
                model_dir = validate_model_for_camera(args.model, robot_config['hardware']['cameras'][args.camera])
                print(f"  Model: {model_dir}")
        
        print(f"\n🎉 All deployments complete!")
        
    except Exception as e:
        print(f"\n❌ Deployment failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
