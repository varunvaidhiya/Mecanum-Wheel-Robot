# VLA Data Engine - Complete Implementation Guide for Windsurf/Cursor IDE

## 🎯 Project Goal
Build a complete Vision-Language-Action (VLA) data engine for a mecanum wheel robot project. This engine will convert ROS2 bag files into an optimized HDF5 format suitable for training VLA models, provide PyTorch dataloaders, and include visualization/validation tools.

---

## 📋 Implementation Checklist

### Phase 1: Data Schema & Storage (Week 1)
- [ ] Define HDF5 schema structure with proper data types
- [ ] Implement JPEG compression utilities for images
- [ ] Create schema validation functions
- [ ] Write unit tests for schema operations

### Phase 2: ROS Bag Ingestion (Week 2)
- [ ] Build ROS2 bag parser using `rosbags` library
- [ ] Implement topic synchronization logic
- [ ] Create main ingestion script with CLI
- [ ] Add progress tracking and error handling

### Phase 3: PyTorch Integration (Week 3)
- [ ] Implement VLADataset class
- [ ] Create data augmentation transforms
- [ ] Build custom collate functions
- [ ] Test with PyTorch DataLoader

### Phase 4: Visualization Tools (Week 4)
- [ ] Create interactive episode viewer
- [ ] Build dataset statistics plotter
- [ ] Add export functionality

### Phase 5: Testing & Documentation (Week 5)
- [ ] Write comprehensive test suite
- [ ] Create CLI batch processing tools
- [ ] Performance benchmarking
- [ ] Complete documentation

---

## 🏗️ Project Structure

Create this exact directory structure:

```
data_engine/
├── __init__.py
├── requirements.txt
├── setup.py
│
├── schema/
│   ├── __init__.py
│   ├── format.py           # HDF5 structure definition
│   ├── constants.py        # Data specifications
│   └── validator.py        # Schema validation
│
├── ingestion/
│   ├── __init__.py
│   ├── bag_to_hdf5.py     # Main CLI ingestion tool
│   ├── ros_parser.py      # ROS bag parsing
│   ├── sync_topics.py     # Timestamp synchronization
│   └── compression.py     # Image compression
│
├── loader/
│   ├── __init__.py
│   ├── dataset.py         # PyTorch Dataset
│   ├── transforms.py      # Data augmentation
│   └── collate.py         # Batch collation
│
├── visualization/
│   ├── __init__.py
│   ├── visualize_episode.py  # Interactive viewer
│   ├── plot_statistics.py    # Dataset analysis
│   └── render_utils.py       # Helper functions
│
├── utils/
│   ├── __init__.py
│   ├── io.py              # File I/O utilities
│   └── timing.py          # Timestamp helpers
│
└── config/
    ├── default_config.yaml
    └── topic_mappings.yaml

tests/
├── __init__.py
├── test_schema.py
├── test_ingestion.py
├── test_dataset.py
└── conftest.py

scripts/
├── ingest_dataset.py       # Batch processing
├── validate_dataset.py     # Validation tool
├── export_samples.py       # Export utilities
└── benchmark_loader.py     # Performance testing

docs/
├── schema_specification.md
├── usage_guide.md
└── API_reference.md
```

---

## 📦 Dependencies

Create `data_engine/requirements.txt`:

```txt
# Core data processing
h5py>=3.10.0
numpy>=1.24.0
opencv-python>=4.8.0

# ROS integration (pure Python - no ROS installation needed)
rosbags>=0.9.16

# Deep learning
torch>=2.1.0
torchvision>=0.16.0

# Visualization
matplotlib>=3.8.0
pillow>=10.0.0
tqdm>=4.66.0

# CLI and utilities
pyyaml>=6.0
click>=8.1.0
pytest>=7.4.0
```

---

## 🗄️ HDF5 Schema Specification

### File: `data_engine/schema/format.py`

**CRITICAL: This is the foundation of your data engine. Get this right first.**

```python
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
```

---

### File: `data_engine/schema/constants.py`

```python
"""Data specifications and constants"""

from dataclasses import dataclass
from typing import List, Tuple
import numpy as np

@dataclass
class StateSpec:
    """Robot state specification"""
    names: List[str]
    ranges: List[Tuple[float, float]]  # (min, max)
    units: List[str]
    dim: int

# State: [x, y, theta, vel_x, vel_y, omega, motor_fl, motor_fr, motor_bl, motor_br]
STATE_SPEC = StateSpec(
    names=['x', 'y', 'theta', 'vel_x', 'vel_y', 'omega', 
           'motor_fl', 'motor_fr', 'motor_bl', 'motor_br'],
    ranges=[
        (-10.0, 10.0),   # x (m)
        (-10.0, 10.0),   # y (m)
        (-np.pi, np.pi), # theta (rad)
        (-2.0, 2.0),     # vel_x (m/s)
        (-2.0, 2.0),     # vel_y (m/s)
        (-3.0, 3.0),     # omega (rad/s)
        (-255, 255),     # motor FL PWM
        (-255, 255),     # motor FR PWM
        (-255, 255),     # motor BL PWM
        (-255, 255),     # motor BR PWM
    ],
    units=['m', 'm', 'rad', 'm/s', 'm/s', 'rad/s', 
           'pwm', 'pwm', 'pwm', 'pwm'],
    dim=10
)

@dataclass
class ActionSpec:
    """Robot action specification"""
    names: List[str]
    ranges: List[Tuple[float, float]]
    units: List[str]
    dim: int

# Action: [linear_x, linear_y, angular_z]
ACTION_SPEC = ActionSpec(
    names=['linear_x', 'linear_y', 'angular_z'],
    ranges=[
        (-2.0, 2.0),   # linear_x (m/s)
        (-2.0, 2.0),   # linear_y (m/s)
        (-3.0, 3.0),   # angular_z (rad/s)
    ],
    units=['m/s', 'm/s', 'rad/s'],
    dim=3
)

@dataclass
class CameraConfig:
    """Camera configuration"""
    name: str
    topic: str
    resolution: Tuple[int, int]  # (height, width)
    fps: int
    encoding: str

DEFAULT_CAMERAS = [
    CameraConfig('front', '/camera/front/image_raw/compressed', (480, 640), 30, 'rgb8'),
    CameraConfig('wrist', '/camera/wrist/image_raw/compressed', (480, 640), 30, 'rgb8'),
]
```

---

### File: `data_engine/schema/validator.py`

```python
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
```

---

## 🔄 ROS Bag Ingestion Pipeline

### File: `data_engine/ingestion/ros_parser.py`

**IMPORTANT: This uses `rosbags` library - pure Python, no ROS installation required**

```python
"""ROS2 bag parsing utilities"""

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import numpy as np
from typing import Dict, List, Tuple, Any
from pathlib import Path

class ROSBagParser:
    """Parse ROS2 bag files and extract data"""
    
    def __init__(self, bag_path: str, topic_config: Dict):
        """
        Args:
            bag_path: Path to ROS2 bag directory
            topic_config: Dict mapping data types to topics
                Example:
                {
                    'cameras': {
                        'front': '/camera/front/image_raw/compressed',
                        'wrist': '/camera/wrist/image_raw/compressed'
                    },
                    'state': '/robot/odom',
                    'cmd_vel': '/cmd_vel'
                }
        """
        self.bag_path = Path(bag_path)
        self.topic_config = topic_config
        self.reader = None
    
    def __enter__(self):
        self.reader = Reader(self.bag_path)
        self.reader.open()
        return self
    
    def __exit__(self, *args):
        if self.reader:
            self.reader.close()
    
    def get_topic_messages(self, topic: str) -> List[Tuple[int, Any]]:
        """
        Extract all messages from a topic.
        
        Returns:
            List of (timestamp_nanoseconds, deserialized_message) tuples
        """
        messages = []
        
        # Get connections for this topic
        connections = [c for c in self.reader.connections if c.topic == topic]
        
        if not connections:
            print(f"Warning: No data found for topic {topic}")
            return messages
        
        # Read messages
        for connection, timestamp, rawdata in self.reader.messages(
            connections=connections
        ):
            msg = deserialize_cdr(rawdata, connection.msgtype)
            messages.append((timestamp, msg))
        
        return messages
    
    def decode_compressed_image(self, msg) -> np.ndarray:
        """
        Decode CompressedImage message to numpy array.
        
        Args:
            msg: sensor_msgs/CompressedImage message
            
        Returns:
            (H, W, 3) RGB uint8 array
        """
        import cv2
        
        # Decode JPEG/PNG
        nparr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if img is None:
            raise RuntimeError("Failed to decode image")
        
        # Convert BGR to RGB
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return rgb
    
    def decode_image(self, msg) -> np.ndarray:
        """
        Decode raw Image message to numpy array.
        
        Args:
            msg: sensor_msgs/Image message
            
        Returns:
            (H, W, 3) RGB uint8 array
        """
        import cv2
        
        # Reshape according to image dimensions
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape((msg.height, msg.width, -1))
        
        # Handle encoding
        if msg.encoding == 'bgr8':
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        elif msg.encoding == 'rgb8':
            pass  # Already RGB
        else:
            raise ValueError(f"Unsupported encoding: {msg.encoding}")
        
        return img
    
    def parse_odometry(self, msg) -> np.ndarray:
        """
        Parse Odometry message to state vector.
        
        Args:
            msg: nav_msgs/Odometry message
            
        Returns:
            State array [x, y, theta, vx, vy, omega, 0, 0, 0, 0]
            Note: Motor values filled with zeros (get from motor topic)
        """
        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Orientation (quaternion to yaw)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert to Euler angle (yaw)
        import math
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        # Velocity
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        # State vector (motors filled separately)
        state = np.array([x, y, theta, vx, vy, omega, 0, 0, 0, 0], 
                        dtype=np.float32)
        return state
    
    def parse_twist(self, msg) -> np.ndarray:
        """
        Parse Twist message (cmd_vel) to action vector.
        
        Returns:
            [linear_x, linear_y, angular_z]
        """
        action = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.angular.z
        ], dtype=np.float32)
        return action
    
    def extract_all_data(self) -> Dict[str, List[Tuple]]:
        """
        Extract all relevant data from bag.
        
        Returns:
            {
                'cameras': {
                    'front': [(timestamp, image), ...],
                    'wrist': [(timestamp, image), ...]
                },
                'states': [(timestamp, state_array), ...],
                'actions': [(timestamp, action_array), ...]
            }
        """
        from tqdm import tqdm
        
        data = {
            'cameras': {},
            'states': [],
            'actions': []
        }
        
        # Extract camera data
        if 'cameras' in self.topic_config:
            for cam_name, topic in self.topic_config['cameras'].items():
                print(f"Extracting {cam_name} camera from {topic}...")
                messages = self.get_topic_messages(topic)
                
                images = []
                for ts, msg in tqdm(messages, desc=f"Decoding {cam_name}"):
                    # Handle both compressed and raw images
                    if hasattr(msg, 'format'):  # CompressedImage
                        img = self.decode_compressed_image(msg)
                    else:  # Raw Image
                        img = self.decode_image(msg)
                    images.append((ts, img))
                
                data['cameras'][cam_name] = images
        
        # Extract state
        if 'state' in self.topic_config:
            print(f"Extracting state from {self.topic_config['state']}...")
            messages = self.get_topic_messages(self.topic_config['state'])
            for ts, msg in tqdm(messages, desc="Parsing odometry"):
                state = self.parse_odometry(msg)
                data['states'].append((ts, state))
        
        # Extract actions
        if 'cmd_vel' in self.topic_config:
            print(f"Extracting actions from {self.topic_config['cmd_vel']}...")
            messages = self.get_topic_messages(self.topic_config['cmd_vel'])
            for ts, msg in tqdm(messages, desc="Parsing cmd_vel"):
                action = self.parse_twist(msg)
                data['actions'].append((ts, action))
        
        return data
```

---

### File: `data_engine/ingestion/sync_topics.py`

```python
"""Timestamp synchronization for multi-modal data"""

import numpy as np
from typing import Dict, List, Tuple, Any
from scipy.interpolate import interp1d

class TopicSynchronizer:
    """Synchronize multi-modal sensor data by timestamp"""
    
    def __init__(self, target_fps: float = 10.0, sync_tolerance: float = 0.05):
        """
        Args:
            target_fps: Output frequency in Hz
            sync_tolerance: Max time difference for matching (seconds)
        """
        self.target_fps = target_fps
        self.sync_tolerance = sync_tolerance
    
    def synchronize(self, data_streams: Dict[str, List[Tuple]]) -> List[Dict]:
        """
        Synchronize multiple data streams to common timestamps.
        
        Args:
            data_streams: {
                'camera_front': [(ts_ns, img), ...],
                'camera_wrist': [(ts_ns, img), ...],
                'state': [(ts_ns, state), ...],
                'action': [(ts_ns, action), ...]
            }
        
        Returns:
            List of synchronized frames, each containing all modalities
        """
        # Convert nanoseconds to seconds
        data_sec = {}
        for key, data in data_streams.items():
            if len(data) > 0:
                timestamps = np.array([ts * 1e-9 for ts, _ in data])
                values = [val for _, val in data]
                data_sec[key] = (timestamps, values)
        
        # Find common time range
        start_time = max(ts[0] for ts, _ in data_sec.values())
        end_time = min(ts[-1] for ts, _ in data_sec.values())
        
        # Generate target timestamps
        dt = 1.0 / self.target_fps
        target_timestamps = np.arange(start_time, end_time, dt)
        
        print(f"Synchronizing {len(target_timestamps)} frames from "
              f"{start_time:.2f}s to {end_time:.2f}s")
        
        # Synchronize each stream
        synchronized = []
        
        for target_ts in target_timestamps:
            frame = {'timestamp': target_ts}
            
            for key, (timestamps, values) in data_sec.items():
                # Find nearest data point
                idx = np.argmin(np.abs(timestamps - target_ts))
                
                # Check if within tolerance
                if abs(timestamps[idx] - target_ts) > self.sync_tolerance:
                    print(f"Warning: No match within tolerance for {key} "
                          f"at t={target_ts:.3f}s")
                    continue
                
                frame[key] = values[idx]
            
            # Only add frame if all required data present
            required_keys = set(data_sec.keys())
            if required_keys.issubset(frame.keys()):
                synchronized.append(frame)
        
        print(f"Successfully synchronized {len(synchronized)} frames")
        return synchronized
```

---

### File: `data_engine/ingestion/bag_to_hdf5.py`

**MAIN INGESTION SCRIPT - This is the primary CLI tool**

```python
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