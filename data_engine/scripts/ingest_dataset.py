#!/usr/bin/env python3
"""
Batch process multiple ROS bags into a single or multiple HDF5 datasets.
"""

import click
import os
import subprocess
from pathlib import Path
from tqdm import tqdm
import multiprocessing

@click.command()
@click.option('--input-dir', '-i', required=True, type=click.Path(exists=True),
              help='Directory containing multiple ROS bag directories')
@click.option('--output-dir', '-o', required=True, type=click.Path(),
              help='Directory to save HDF5 files')
@click.option('--workers', '-w', default=1, help='Number of parallel workers')
def batch_ingest(input_dir, output_dir, workers):
    """Batch convert all ROS bags in a directory."""
    
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Find all bag directories (assuming metadata.yaml exists)
    bags = sorted([p.parent for p in input_path.rglob('metadata.yaml')])
    
    print(f"Found {len(bags)} bags in {input_dir}")
    
    commands = []
    for bag in bags:
        # Create output filename based on bag name
        bag_name = bag.name
        h5_name = f"{bag_name}.h5"
        out_file = output_path / h5_name
        
        # Construct command
        cmd = [
            'python', '-m', 'data_engine.ingestion.bag_to_hdf5',
            '--input', str(bag),
            '--output', str(out_file),
            '--episode-name', bag_name
        ]
        commands.append((cmd, bag_name))

    # Run processing
    if workers > 1:
        with multiprocessing.Pool(workers) as pool:
            results = pool.map(run_command, commands)
    else:
        for cmd, name in tqdm(commands, desc="Processing bags"):
            run_command((cmd, name))

def run_command(args):
    cmd, name = args
    try:
        subprocess.run(cmd, check=True, capture_output=True, text=True)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error processing {name}: {e.stderr}")
        return False

if __name__ == '__main__':
    batch_ingest()
