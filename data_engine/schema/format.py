"""
HDF5 Schema for VLA Training Data

Directory Structure:
/metadata                          # Global dataset metadata
    - dataset_name: str
    - creation_date: str
    - total_episodes: int
    - total_timesteps: int
    - fps: float
    - robot_type: str

/episode_000000/                   # Individual episode
    /observations/
        /images/
            /front (N,) vlen(uint8)     # JPEG compressed
            /wrist (N,) vlen(uint8)     # JPEG compressed
            /third_person (N,) vlen(uint8)  # Optional
        /state (N, 10) float32          # Robot state vector
        /timestamps (N,) float64        # Unix timestamps
    /actions/
        /cmd_vel (N, 3) float32         # [linear_x, linear_y, angular_z]
        /motor_commands (N, 4) int16    # [pwm_fl, pwm_fr, pwm_bl, pwm_br]
    /language/
        /instruction (str)              # Task description
        /sub_instructions (N,) str      # Per-frame annotations (optional)
    /metadata/
        - episode_length: int
        - success: bool
        - environment: str
        - fps: float

/episode_000001/
    ... (same structure)
"""

import h5py
import numpy as np
from typing import Dict, List, Optional
import cv2

# ===== CONSTANTS =====
COMPRESSION_QUALITY = 95
CHUNK_SIZE = 100
STATE_DIM = 10
ACTION_DIM = 3
MOTOR_DIM = 4
DEFAULT_IMAGE_SHAPE = (480, 640, 3)
SUPPORTED_CAMERAS = ['front', 'wrist', 'third_person']

# Data types
IMAGE_DTYPE = np.uint8
STATE_DTYPE = np.float32
ACTION_DTYPE = np.float32
TIMESTAMP_DTYPE = np.float64

def create_episode_group(
    hdf5_file: h5py.File,
    episode_name: str,
    episode_length: int,
    fps: float = 10.0
) -> h5py.Group:
    """
    Create a new episode group with proper structure.
    
    Args:
        hdf5_file: Open HDF5 file handle
        episode_name: Name like 'episode_000000'
        episode_length: Number of timesteps
        fps: Frames per second
        
    Returns:
        Episode group handle
    """
    ep = hdf5_file.create_group(episode_name)
    
    # Create subgroups
    obs = ep.create_group('observations')
    obs.create_group('images')
    actions = ep.create_group('actions')
    lang = ep.create_group('language')
    meta = ep.create_group('metadata')
    
    # Set metadata
    meta.attrs['episode_length'] = episode_length
    meta.attrs['fps'] = fps
    
    return ep

def add_compressed_images(
    episode_group: h5py.Group,
    camera_name: str,
    images: List[np.ndarray],
    quality: int = COMPRESSION_QUALITY
) -> None:
    """
    Add compressed images to episode.
    
    Args:
        episode_group: Episode HDF5 group
        camera_name: e.g., 'front', 'wrist'
        images: List of (H, W, 3) RGB uint8 arrays
        quality: JPEG quality 0-100
    """
    # Compress all images
    compressed = []
    for img in images:
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        success, encoded = cv2.imencode(
            '.jpg', bgr, 
            [cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        if not success:
            raise RuntimeError(f"Failed to encode image")
        compressed.append(encoded.tobytes())
    
    # Store as variable-length dataset
    img_group = episode_group['observations']['images']
    dt = h5py.vlen_dtype(np.dtype('uint8'))
    dset = img_group.create_dataset(
        camera_name,
        (len(compressed),),
        dtype=dt,
        compression='gzip',
        compression_opts=4
    )
    dset[:] = compressed
    dset.attrs['compression'] = 'jpeg'
    dset.attrs['quality'] = quality
    dset.attrs['original_shape'] = images[0].shape

def decompress_image(jpeg_bytes: bytes) -> np.ndarray:
    """
    Decompress JPEG bytes to RGB image.
    
    Args:
        jpeg_bytes: JPEG encoded bytes
        
    Returns:
        (H, W, 3) RGB uint8 array
    """
    nparr = np.frombuffer(jpeg_bytes, np.uint8)
    bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError("Failed to decode JPEG")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return rgb
