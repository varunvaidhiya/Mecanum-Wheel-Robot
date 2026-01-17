"""Test VLADataset"""

import pytest
import torch
import numpy as np
import h5py
import tempfile
import os
from data_engine.loader.dataset import VLADataset
from data_engine.schema.format import create_episode_group, add_compressed_images

def create_dummy_dataset(path):
    with h5py.File(path, 'w') as f:
        f.create_group('metadata').attrs['dataset_name'] = 'test'
        ep = create_episode_group(f, 'episode_000000', 5, 10.0)
        
        # Add images
        imgs = [np.zeros((100, 100, 3), dtype=np.uint8) for _ in range(5)]
        add_compressed_images(ep, 'front', imgs)
        
        # Add state
        state = np.zeros((5, 10), dtype=np.float32)
        ep['observations'].create_dataset('state', data=state)
        
        # Add action
        action = np.zeros((5, 3), dtype=np.float32)
        ep['actions'].create_dataset('cmd_vel', data=action)
        
        # Add lang
        ep['language'].create_dataset('instruction', data='test task')

def test_dataset_loading():
    """Test loading data from VLADataset"""
    
    with tempfile.NamedTemporaryFile(suffix='.h5', delete=False) as tmp:
        path = tmp.name
        
    try:
        create_dummy_dataset(path)
        
        dataset = VLADataset(path, cameras=['front'])
        assert len(dataset) == 5
        
        sample = dataset[0]
        assert 'images' in sample
        assert 'front' in sample['images']
        assert sample['images']['front'].shape == (3, 100, 100) # C, H, W
        
        assert 'state' in sample
        assert sample['state'].shape == (10,)
        
        assert 'action' in sample
        assert sample['action'].shape == (3,)
        
        assert sample['instruction'] == 'test task'
        
    finally:
        if os.path.exists(path):
            os.remove(path)

if __name__ == "__main__":
    test_dataset_loading()
