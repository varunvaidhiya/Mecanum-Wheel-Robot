#!/usr/bin/env python3
"""
Interactive HDF5 Episode Visualizer.

Usage:
    python visualize_episode.py --path dataset.h5 --episode episode_000000
"""

import click
import h5py
import cv2
import numpy as np
import time
from pathlib import Path
from typing import Dict, List

from data_engine.schema.format import decompress_image

@click.command()
@click.option('--path', '-p', required=True, type=click.Path(exists=True),
              help='Path to HDF5 dataset')
@click.option('--episode', '-e', default=None,
              help='Episode name (default: first episode)')
@click.option('--fps', default=10.0, help='Playback FPS')
def visualize(path, episode, fps):
    """Visualize an episode from the VLA dataset."""
    
    print(f"Opening {path}...")
    
    with h5py.File(path, 'r') as f:
        # Select episode
        if episode is None:
            # Find first episode
            episodes = sorted([k for k in f.keys() if k.startswith('episode_')])
            if not episodes:
                print("No episodes found in file!")
                return
            episode = episodes[0]
            
        if episode not in f:
            print(f"Episode '{episode}' not found!")
            print(f"Available episodes: {list(f.keys())}")
            return
            
        print(f"Visualizing {episode}...")
        ep = f[episode]
        
        # Load data
        obs = ep['observations']
        actions = ep['actions']
        
        # Get episode length
        length = ep['metadata'].attrs['episode_length']
        print(f"Episode length: {length} frames")
        
        # Get available cameras
        cameras = list(obs['images'].keys())
        print(f"Cameras: {cameras}")
        
        # Load other data
        states = obs['state'][:]
        cmd_vel = actions['cmd_vel'][:]
        instruction = ep['language']['instruction'][()].decode('utf-8')
        
        print(f"Instruction: {instruction}")
        
        # Prepare windows
        for cam in cameras:
            cv2.namedWindow(f"Camera: {cam}", cv2.WINDOW_NORMAL)
            
        print("\nControls:")
        print("  SPACE: Pause/Play")
        print("  q:     Quit")
        print("  n:     Next frame (when paused)")
        
        paused = False
        idx = 0
        
        while idx < length:
            start_time = time.time()
            
            # 1. Get images
            display_imgs = {}
            for cam in cameras:
                # Keep file open, read compressed bytes
                compressed_bytes = obs['images'][cam][idx]
                img = decompress_image(compressed_bytes)
                
                # Overlay info
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                display_imgs[cam] = img
            
            # 2. Add text overlay
            # Create a status image or overlay on first camera
            status_text = [
                f"Frame: {idx}/{length}",
                f"State: x={states[idx][0]:.2f}, y={states[idx][1]:.2f}, th={states[idx][2]:.2f}",
                f"Action: vx={cmd_vel[idx][0]:.2f}, vy={cmd_vel[idx][1]:.2f}, w={cmd_vel[idx][2]:.2f}",
                f"Instr: {instruction}"
            ]
            
            if cameras:
                main_cam = cameras[0]
                img = display_imgs[main_cam]
                y0, dy = 30, 25
                for i, line in enumerate(status_text):
                    y = y0 + i * dy
                    cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.6, (0, 255, 0), 2)
            
            # 3. Show images
            for cam, img in display_imgs.items():
                cv2.imshow(f"Camera: {cam}", img)
            
            # 4. Handle Input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):
                paused = not paused
            elif key == ord('n') and paused:
                idx += 1
            
            if not paused:
                idx += 1
                
                # Control FPS
                dt = time.time() - start_time
                target_dt = 1.0 / fps
                if dt < target_dt:
                    time.sleep(target_dt - dt)
        
        cv2.destroyAllWindows()
        print("End of episode.")

if __name__ == '__main__':
    visualize()
