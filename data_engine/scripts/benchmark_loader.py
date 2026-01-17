#!/usr/bin/env python3
"""
Benchmark PyTorch DataLoader performance.
"""

import click
import time
import torch
from torch.utils.data import DataLoader
from tqdm import tqdm
from data_engine.loader.dataset import VLADataset

@click.command()
@click.option('--path', required=True, help='Path to dataset')
@click.option('--batch-size', default=32, help='Batch size')
@click.option('--workers', default=4, help='Number of workers')
@click.option('--iterations', default=100, help='Number of batches to load')
def benchmark(path, batch_size, workers, iterations):
    """Benchmark data loading speed."""
    
    print(f"Benchmarking loader on {path}")
    print(f"Batch: {batch_size}, Workers: {workers}")
    
    try:
        dataset = VLADataset(path)
    except Exception as e:
        print(f"Failed to load dataset: {e}")
        return
        
    loader = DataLoader(
        dataset, 
        batch_size=batch_size, 
        num_workers=workers,
        shuffle=True
    )
    
    print(f"Dataset size: {len(dataset)}")
    
    # Warmup
    print("Warming up...")
    iter_loader = iter(loader)
    next(iter_loader)
    
    # Benchmark
    print("Running benchmark...")
    start = time.time()
    
    for _ in tqdm(range(iterations)):
        try:
            batch = next(iter_loader)
        except StopIteration:
            iter_loader = iter(loader)
            batch = next(iter_loader)
            
    end = time.time()
    
    duration = end - start
    total_samples = iterations * batch_size
    fps = total_samples / duration
    
    print("\nResults:")
    print(f"  Time: {duration:.2f}s")
    print(f"  Samples: {total_samples}")
    print(f"  FPS: {fps:.2f} samples/sec")

if __name__ == '__main__':
    benchmark()
