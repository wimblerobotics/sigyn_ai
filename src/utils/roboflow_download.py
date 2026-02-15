#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Sigyn AI Contributors
"""
RoboFlow integration script for Sigyn AI.

This script automates:
1. Listing available RoboFlow projects and versions
2. Downloading datasets with proper directory structure
3. Tracking dataset versions and metadata

Usage:
    # List projects
    python roboflow_download.py --list-projects
    
    # List versions of a project
    python roboflow_download.py --project FCC4 --list-versions
    
    # Download specific version
    python roboflow_download.py --project FCC4 --version 4 --format yolov8
    
    # Download latest version
    python roboflow_download.py --project FCC4 --format yolov8
"""

import argparse
import sys
import os
import json
from pathlib import Path
from datetime import datetime


def check_roboflow_installed():
    """Check if roboflow package is installed."""
    try:
        from roboflow import Roboflow
        return True
    except ImportError:
        print("❌ RoboFlow SDK not installed")
        print("\n📦 Install with:")
        print("   pip install roboflow")
        return False


def get_api_key():
    """Get RoboFlow API key from environment or prompt user."""
    api_key = os.environ.get('ROBOFLOW_API_KEY')
    
    if not api_key:
        print("⚠️  ROBOFLOW_API_KEY not found in environment")
        print("\n🔑 Get your API key:")
        print("   1. Go to: https://app.roboflow.com/settings/api")
        print("   2. Copy your API key")
        print("   3. Set environment variable:")
        print("      export ROBOFLOW_API_KEY='your_key_here'")
        print("   4. Or add to ~/.bashrc for persistence")
        
        api_key = input("\n🔑 Enter API key now (or press Enter to exit): ").strip()
        
        if not api_key:
            sys.exit(1)
    
    return api_key


def list_projects(rf):
    """List all available RoboFlow projects."""
    try:
        workspace = rf.workspace()
        projects = workspace.projects()
        
        print(f"\n📋 Available projects in workspace:")
        print(f"{'Project ID':<25} {'Name':<30} {'Images':<10} {'Versions':<10}")
        print("-" * 80)
        
        for project_id in project_list:
            # Get full project details
            try:
                project = workspace.project(project_id)
                project_name = project.name if hasattr(project, 'name') else project_id
                images = project.annotation if hasattr(project, 'annotation') else 'N/A'
                versions = len(project.versions()) if hasattr(project, 'versions') else 'N/A'
                
                print(f"{project_id:<25} {project_name:<30} {str(images):<10} {str(versions):<10}")
            except Exception:
                # Just list the ID if we can't get details
                print(f"{project_id:<25} {'N/A':<30} {'N/A':<10} {'N/A':<10}")
        
        print("\n💡 Tip: Use the Project ID (first column) with --project parameter")
        print("   Example: python src/utils/roboflow_download.py --project fcc4 --format yolov8")
        
        return project_list
    except Exception as e:
        print(f"❌ Failed to list projects: {e}")
        import traceback
        traceback.print_exc()
        return []


def list_versions(rf, project_name):
    """List all versions of a project."""
    try:
        project = rf.workspace().project(project_name)
        versions = project.versions()
        
        print(f"\n📋 Available versions for {project_name}:")
        for version in versions:
            print(f"   - Version {version.version}")
            print(f"     Name: {version.name}")
            print(f"     Created: {version.created}")
            print(f"     Images: {version.splits}")
        
        return versions
    except Exception as e:
        print(f"❌ Failed to list versions: {e}")
        return []


def download_dataset(rf, project_name, version_number, format_type, output_base):
    """Download dataset from RoboFlow."""
    try:
        # Get project - try case-insensitive match first
        workspace = rf.workspace()
        project = None
        
        # First, try exact match
        try:
            project = workspace.project(project_name)
        except Exception:
            # Try lowercase version
            try:
                project = workspace.project(project_name.lower())
                print(f"💡 Using lowercase project ID: {project_name.lower()}")
            except Exception as e:
                # List available projects to help user
                print(f"\n❌ Project '{project_name}' not found")
                print("\n📋 Available projects:")
                project_list = workspace.project_list
                for proj_id in project_list:
                    try:
                        p = workspace.project(proj_id)
                        print(f"   - {proj_id} ({p.name if hasattr(p, 'name') else 'N/A'})")
                    except:
                        print(f"   - {proj_id}")
                raise ValueError(f"Project '{project_name}' not found. Use exact Project ID from list above (case-sensitive).")
        
        # Get version (latest if not specified)
        if version_number:
            version = project.version(version_number)
        else:
            versions = project.versions()
            version = versions[0] if versions else None
            if not version:
                raise ValueError("No versions found for project")
            version_number = version.version
            print(f"📥 Using latest version: {version_number}")
        
        # Determine output directory (don't create it yet - let RoboFlow do that)
        output_dir = Path(output_base) / f"{project_name}.v{version_number}-{format_type}"
        
        # Ensure parent directory exists
        output_dir.parent.mkdir(parents=True, exist_ok=True)
        
        print(f"\n📥 Downloading {project_name} v{version_number}...")
        print(f"   Format: {format_type}")
        print(f"   Output: {output_dir}")
        
        # Download dataset
        dataset = version.download(
            model_format=format_type,
            location=str(output_dir)
        )
        
        # Verify download
        # Check the dataset location first
        if hasattr(dataset, 'location'):
            actual_dir = Path(dataset.location)
        else:
            actual_dir = output_dir
            
        data_yaml = actual_dir / 'data.yaml'
        if not data_yaml.exists():
            print(f"\n⚠️  Warning: data.yaml not found in {actual_dir}")
            # Try to find it anywhere under output_dir
            found_yamls = list(Path(output_dir).rglob('data.yaml'))
            if found_yamls:
                data_yaml = found_yamls[0]
                actual_dir = data_yaml.parent
                print(f"   ✓ Found data.yaml in: {actual_dir}")
            else:
                # List what files were actually downloaded
                all_files = list(Path(output_dir).rglob('*'))
                print(f"   Files found in {output_dir}:")
                for f in all_files[:10]:  # Show first 10
                    print(f"     - {f}")
                raise FileNotFoundError(f"data.yaml not found in {output_dir}. Download may have failed.")
        
        # Update output_dir to actual location if different
        output_dir = actual_dir
        
        # Save metadata
        metadata = {
            'project_name': project_name,
            'version': version_number,
            'format': format_type,
            'download_date': datetime.now().isoformat(),
            'images': version.splits,
            'created': version.created,
            'roboflow_url': f"https://app.roboflow.com/{rf.workspace().url}/datasets/{project_name}/{version_number}"
        }
        
        metadata_path = output_dir / 'dataset_metadata.json'
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        print(f"\n✅ Dataset downloaded successfully!")
        print(f"   Location: {output_dir}")
        print(f"   Metadata: {metadata_path}")
        print(f"\n📊 Dataset info:")
        print(f"   Train images: {version.splits.get('train', 'N/A')}")
        print(f"   Valid images: {version.splits.get('valid', 'N/A')}")
        print(f"   Test images: {version.splits.get('test', 'N/A')}")
        
        # Print next steps
        print(f"\n🚀 Next steps:")
        print(f"   1. Train model:")
        print(f"      python src/training/train.py --config configs/training/<config_name>.yaml")
        print(f"   2. Update config to use dataset:")
        print(f"      dataset:")
        print(f"        path: \"{output_dir}\"")
        
        return output_dir
        
    except Exception as e:
        print(f"❌ Download failed: {e}")
        import traceback
        traceback.print_exc()
        return None


def main():
    parser = argparse.ArgumentParser(
        description='Download datasets from RoboFlow',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # List all projects
  python roboflow_download.py --list-projects
  
  # List versions of a project
  python roboflow_download.py --project FCC4 --list-versions
  
  # Download specific version for YOLOv8
  python roboflow_download.py --project FCC4 --version 4 --format yolov8
  
  # Download latest version for OAK-D (YOLOv5 format)
  python roboflow_download.py --project FCC4 --format yolov5

Environment:
  ROBOFLOW_API_KEY - Your RoboFlow API key (required)
  Get from: https://app.roboflow.com/settings/api
        """
    )
    
    parser.add_argument(
        '--list-projects',
        action='store_true',
        help='List all available projects'
    )
    
    parser.add_argument(
        '--list-versions',
        action='store_true',
        help='List all versions of a project'
    )
    
    parser.add_argument(
        '--project',
        type=str,
        help='RoboFlow project name'
    )
    
    parser.add_argument(
        '--version',
        type=int,
        default=None,
        help='Dataset version number (default: latest)'
    )
    
    parser.add_argument(
        '--format',
        type=str,
        default='yolov8',
        choices=['yolov8', 'yolov5', 'coco', 'createml', 'tfrecord'],
        help='Export format (default: yolov8)'
    )
    
    parser.add_argument(
        '--output',
        type=str,
        default='datasets/roboflow_exports',
        help='Output directory (default: datasets/roboflow_exports)'
    )
    
    args = parser.parse_args()
    
    # Check if roboflow is installed
    if not check_roboflow_installed():
        sys.exit(1)
    
    # Get API key
    api_key = get_api_key()
    
    # Initialize RoboFlow
    try:
        from roboflow import Roboflow
        rf = Roboflow(api_key=api_key)
        print("✅ Connected to RoboFlow")
    except Exception as e:
        print(f"❌ Failed to connect to RoboFlow: {e}")
        sys.exit(1)
    
    # Execute requested action
    try:
        if args.list_projects:
            list_projects(rf)
        
        elif args.list_versions:
            if not args.project:
                print("❌ --project required with --list-versions")
                sys.exit(1)
            list_versions(rf, args.project)
        
        else:
            # Download dataset
            if not args.project:
                print("❌ --project required for download")
                sys.exit(1)
            
            download_dataset(rf, args.project, args.version, args.format, args.output)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        sys.exit(1)


if __name__ == '__main__':
    main()
