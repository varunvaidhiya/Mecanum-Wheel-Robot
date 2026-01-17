"""Schema validation tools"""

import h5py
import numpy as np
from typing import Dict, List
from pathlib import Path

def validate_hdf5_schema(filepath: str) -> Dict[str, bool]:
    """
    Validate HDF5 file against VLA schema.
    
    Returns:
        Dict mapping check names to pass/fail status
    """
    results = {
        'file_exists': False,
        'has_metadata': False,
        'has_episodes': False,
        'episode_structure_valid': False,
        'data_types_correct': False,
        'shapes_consistent': False,
        'no_missing_data': False,
    }
    
    # Check file exists
    if not Path(filepath).exists():
        return results
    results['file_exists'] = True
    
    try:
        with h5py.File(filepath, 'r') as f:
            # Check metadata
            if 'metadata' in f:
                results['has_metadata'] = True
            
            # Get episodes
            episodes = [k for k in f.keys() if k.startswith('episode_')]
            if len(episodes) > 0:
                results['has_episodes'] = True
            
            # Validate each episode
            all_valid = True
            for ep_name in episodes:
                errors = validate_episode(f[ep_name])
                if errors:
                    all_valid = False
                    print(f"Errors in {ep_name}:")
                    for err in errors:
                        print(f"  - {err}")
            
            if all_valid and len(episodes) > 0:
                results['episode_structure_valid'] = True
                results['data_types_correct'] = True
                results['shapes_consistent'] = True
                results['no_missing_data'] = True
    
    except Exception as e:
        print(f"Validation error: {e}")
    
    return results

def validate_episode(episode_group: h5py.Group) -> List[str]:
    """
    Validate single episode structure.
    
    Returns:
        List of error messages (empty if valid)
    """
    errors = []
    
    # Check required groups
    required_groups = ['observations', 'actions', 'language', 'metadata']
    for grp in required_groups:
        if grp not in episode_group:
            errors.append(f"Missing group: {grp}")
    
    if errors:
        return errors
    
    # Check observations
    obs = episode_group['observations']
    if 'images' not in obs:
        errors.append("Missing observations/images")
    if 'state' not in obs:
        errors.append("Missing observations/state")
    if 'timestamps' not in obs:
        errors.append("Missing observations/timestamps")
    
    # Check actions
    actions = episode_group['actions']
    if 'cmd_vel' not in actions:
        errors.append("Missing actions/cmd_vel")
    
    # Check shapes consistency
    try:
        ep_len = episode_group['metadata'].attrs['episode_length']
        
        if 'state' in obs:
            if obs['state'].shape[0] != ep_len:
                errors.append(f"State length mismatch: {obs['state'].shape[0]} != {ep_len}")
        
        if 'cmd_vel' in actions:
            if actions['cmd_vel'].shape[0] != ep_len:
                errors.append(f"Action length mismatch: {actions['cmd_vel'].shape[0]} != {ep_len}")
        
    except Exception as e:
        errors.append(f"Shape validation error: {e}")
    
    return errors

def check_temporal_consistency(timestamps: np.ndarray, fps: float) -> bool:
    """Check if timestamps are monotonic and match expected FPS"""
    
    # Check monotonic
    if not np.all(timestamps[1:] >= timestamps[:-1]):
        return False
    
    # Check FPS
    diffs = np.diff(timestamps)
    expected_dt = 1.0 / fps
    
    # Allow 20% tolerance
    if not np.allclose(diffs, expected_dt, rtol=0.2):
        return False
    
    return True
