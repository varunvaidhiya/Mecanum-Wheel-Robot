"""Test HDF5 Schema Construction"""

import h5py
import numpy as np
import pytest
import os
from pathlib import Path
import tempfile
import cv2

from data_engine.schema.format import (
    create_episode_group, add_compressed_images,
    decompress_image
)

def test_schema_creation():
    """Test creating an episode and writing data"""
    
    # Create temp file
    with tempfile.NamedTemporaryFile(suffix='.h5', delete=False) as tmp:
        path = tmp.name
        
    try:
        # Write
        with h5py.File(path, 'w') as f:
            f.create_group('metadata').attrs['dataset_name'] = 'test_dataset'
            
            # Create episode
            ep = create_episode_group(f, 'episode_000000', 10, fps=10.0)
            
            # Create dummy images
            imgs = [np.zeros((480, 640, 3), dtype=np.uint8) for _ in range(10)]
            # Add some noise to make sure compression works
            for img in imgs:
                img[:] = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
                
            add_compressed_images(ep, 'front', imgs)
            
        # Read
        with h5py.File(path, 'r') as f:
            ep = f['episode_000000']
            
            assert 'observations' in ep
            assert 'images' in ep['observations']
            assert 'front' in ep['observations']['images']
            
            # Check length - HDF5 vlen datasets can be tricky to len(), but shape should work
            assert ep['observations']['images']['front'].shape == (10,)
            
            # Read first image
            compressed = ep['observations']['images']['front'][0]
            img_out = decompress_image(compressed)
            
            assert img_out.shape == (480, 640, 3)
            assert img_out.dtype == np.uint8
            
        print("Schema test passed!")
        
    finally:
        if os.path.exists(path):
            os.remove(path)

if __name__ == "__main__":
    test_schema_creation()
