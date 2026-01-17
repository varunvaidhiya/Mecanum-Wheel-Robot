"""PyTorch Dataset for VLA Training"""

import h5py
import torch
import numpy as np
from torch.utils.data import Dataset
from typing import Dict, List, Tuple, Optional, Callable
from pathlib import Path

from data_engine.schema.format import decompress_image

class VLADataset(Dataset):
    """
    VLA Dataset that reads from HDF5.
    
    Accesses data at the frame level.
    """
    
    def __init__(
        self, 
        root_dir: str, 
        cameras: List[str] = ['front', 'wrist'],
        transform: Optional[Callable] = None,
        max_episodes: Optional[int] = None
    ):
        """
        Args:
            root_dir: Path to HDF5 file (or directory of HDF5 files)
            cameras: List of cameras to load
            transform: Optional encoding/transform function
            max_episodes: Limit number of episodes (for debugging)
        """
        self.file_path = Path(root_dir)
        if not self.file_path.exists():
            raise FileNotFoundError(f"Dataset not found at {self.file_path}")
            
        self.cameras = cameras
        self.transform = transform
        
        # Open file to build index
        # Note: We can't keep one file handle open for all workers if using multiple workers
        # So we'll open it in __getitem__ or using a worker_init_fn
        # For now, we open just to build index and then close
        
        self.index = []  # List of (episode_name, frame_idx)
        
        with h5py.File(self.file_path, 'r') as f:
            episodes = sorted([k for k in f.keys() if k.startswith('episode_')])
            
            if max_episodes:
                episodes = episodes[:max_episodes]
                
            for ep_name in episodes:
                ep_len = f[ep_name]['metadata'].attrs['episode_length']
                for i in range(ep_len):
                    self.index.append((ep_name, i))
                    
        print(f"Loaded VLADataset from {self.file_path}")
        print(f"  Episodes: {len(episodes)}")
        print(f"  Frames:   {len(self.index)}")
        
    def __len__(self):
        return len(self.index)
    
    def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
        """
        Get a single frame.
        
        Returns:
            {
                'images': {
                    'front': (C, H, W) tensor,
                    ...
                },
                'state': (10,) tensor,
                'action': (3,) tensor,
                'instruction': str
            }
        """
        ep_name, frame_idx = self.index[idx]
        
        # Open file locally (thread-safe for DataLoader workers)
        with h5py.File(self.file_path, 'r') as f:
            ep = f[ep_name]
            
            # 1. Load Images
            images = {}
            for cam in self.cameras:
                if cam in ep['observations']['images']:
                    # Decompress
                    compressed = ep['observations']['images'][cam][frame_idx]
                    img = decompress_image(compressed)
                    
                    # Convert to tensor (H, W, C) -> (C, H, W)
                    img = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
                    images[cam] = img
            
            # 2. Load State
            state = torch.from_numpy(ep['observations']['state'][frame_idx])
            
            # 3. Load Action
            action = torch.from_numpy(ep['actions']['cmd_vel'][frame_idx])
            
            # 4. Load Instruction
            instruction = ep['language']['instruction'][()].decode('utf-8')
            
        sample = {
            'images': images,
            'state': state,
            'action': action,
            'instruction': instruction
        }
        
        if self.transform:
            sample = self.transform(sample)
            
        return sample
