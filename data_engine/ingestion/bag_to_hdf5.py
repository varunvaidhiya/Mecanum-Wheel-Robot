#!/usr/bin/env python3
"""
Convert ROS2 bag to HDF5 format for VLA training.

Usage:
    python bag_to_hdf5.py \\
        --input ./rosbag2_data \\
        --output dataset.h5 \\
        --config config/topic_mappings.yaml \\
        --instruction "Navigate to the red box" \\
        --fps 10
"""

import click
import h5py
import yaml
from pathlib import Path
from datetime import datetime
from tqdm import tqdm
import numpy as np

from data_engine.ingestion.ros_parser import ROSBagParser
from data_engine.ingestion.sync_topics import TopicSynchronizer
from data_engine.schema.format import (
    create_episode_group, add_compressed_images,
    STATE_DTYPE, ACTION_DTYPE, TIMESTAMP_DTYPE
)

@click.command()
@click.option('--input', '-i', required=True, type=click.Path(exists=True),
              help='Path to ROS2 bag directory')
@click.option('--output', '-o', required=True, type=click.Path(),
              help='Output HDF5 file path')
@click.option('--config', '-c', type=click.Path(exists=True),
              help='Topic mapping config YAML')
@click.option('--fps', default=10.0, help='Target FPS')
@click.option('--instruction', type=str, default="",
              help='Language instruction for episode')
@click.option('--episode-name', type=str, default=None,
              help='Custom episode name (auto-increment if not provided)')
@click.option('--compression-quality', default=95, type=int,
              help='JPEG quality 0-100')
def ingest_bag(input, output, config, fps, instruction, 
               episode_name, compression_quality):
    """Convert ROS2 bag to HDF5 for VLA training"""
    
    print("="*60)
    print("VLA Data Engine - ROS Bag Ingestion")
    print("="*60)
    print(f"Input:  {input}")
    print(f"Output: {output}")
    print(f"FPS:    {fps}")
    print("="*60)
    
    # Load configuration
    if config:
        with open(config) as f:
            topic_config = yaml.safe_load(f)
    else:
        # Default configuration
        topic_config = {
            'cameras': {
                'front': '/camera/front/image_raw/compressed',
                'wrist': '/camera/wrist/image_raw/compressed'
            },
            'state': '/robot/odom',
            'cmd_vel': '/cmd_vel'
        }
    
    # Parse ROS bag
    print("\n[1/4] Parsing ROS bag...")
    with ROSBagParser(input, topic_config) as parser:
        raw_data = parser.extract_all_data()
    
    # Prepare data for synchronization
    sync_data = {}
    for cam_name, cam_data in raw_data['cameras'].items():
        sync_data[f'camera_{cam_name}'] = cam_data
    sync_data['state'] = raw_data['states']
    sync_data['action'] = raw_data['actions']
    
    # Synchronize
    print("\n[2/4] Synchronizing topics...")
    synchronizer = TopicSynchronizer(target_fps=fps)
    synchronized = synchronizer.synchronize(sync_data)
    
    if len(synchronized) == 0:
        print("ERROR: No synchronized data!")
        return
    
    print(f"✓ Synchronized {len(synchronized)} frames at {fps} Hz")
    
    # Write to HDF5
    print("\n[3/4] Writing to HDF5...")
    mode = 'a' if Path(output).exists() else 'w'
    
    with h5py.File(output, mode) as f:
        # Create/update global metadata
        if 'metadata' not in f:
            meta = f.create_group('metadata')
            meta.attrs['dataset_name'] = Path(output).stem
            meta.attrs['creation_date'] = datetime.now().isoformat()
            meta.attrs['robot_type'] = 'mecanum_wheel'
        
        # Determine episode name
        if episode_name is None:
            existing = [k for k in f.keys() if k.startswith('episode_')]
            episode_num = len(existing)
            episode_name = f'episode_{episode_num:06d}'
        
        # Create episode
        ep_len = len(synchronized)
        ep = create_episode_group(f, episode_name, ep_len, fps)
        
        # Extract images by camera
        camera_images = {}
        for frame in synchronized:
            for key in frame.keys():
                if key.startswith('camera_'):
                    cam_name = key.replace('camera_', '')
                    if cam_name not in camera_images:
                        camera_images[cam_name] = []
                    camera_images[cam_name].append(frame[key])
        
        # Add images
        for cam_name, images in camera_images.items():
            print(f"  Compressing {cam_name}: {len(images)} images...")
            add_compressed_images(ep, cam_name, images, compression_quality)
        
        # Add state
        states = np.array([f['state'] for f in synchronized], dtype=STATE_DTYPE)
        ep['observations'].create_dataset('state', data=states, compression='gzip')
        
        # Add actions
        actions = np.array([f['action'] for f in synchronized], dtype=ACTION_DTYPE)
        ep['actions'].create_dataset('cmd_vel', data=actions, compression='gzip')
        
        # Add timestamps
        timestamps = np.array([f['timestamp'] for f in synchronized], dtype=TIMESTAMP_DTYPE)
        ep['observations'].create_dataset('timestamps', data=timestamps)
        
        # Add language
        ep['language'].create_dataset('instruction', data=instruction)
        
        # Update metadata
        ep['metadata'].attrs['success'] = True
        ep['metadata'].attrs['bag_path'] = str(input)
        
        # Update global metadata
        total_eps = len([k for k in f.keys() if k.startswith('episode_')])
        f['metadata'].attrs['total_episodes'] = total_eps
        f['metadata'].attrs['fps'] = fps
    
    print(f"\n✓ Episode '{episode_name}' written successfully!")
    
    # Validate
    print("\n[4/4] Validating...")
    from data_engine.schema.validator import validate_hdf5_schema
    results = validate_hdf5_schema(output)
    
    if all(results.values()):
        print("✓ Validation PASSED")
    else:
        print("✗ Validation FAILED:")
        for check, passed in results.items():
            if not passed:
                print(f"  - {check}")
    
    print("\n" + "="*60)
    print("DONE!")
    print("="*60)

if __name__ == '__main__':
    ingest_bag()
