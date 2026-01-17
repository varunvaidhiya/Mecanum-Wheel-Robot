#!/usr/bin/env python3
"""
Validate an entire dataset directory or single file.
"""

import click
import glob
from pathlib import Path
from data_engine.schema.validator import validate_hdf5_schema

@click.command()
@click.option('--path', '-p', required=True, type=str,
              help='Path to HDF5 file or directory/pattern')
def validate(path):
    """Validate HDF5 datasets."""
    
    files = []
    if Path(path).is_dir():
        files = sorted(list(Path(path).glob('*.h5')))
    elif '*' in path:
        files = sorted(list(glob.glob(path)))
    else:
        files = [Path(path)]
        
    print(f"Validating {len(files)} files...")
    
    passed_count = 0
    for f in files:
        print(f"\nChecking {f.name}...")
        results = validate_hdf5_schema(str(f))
        
        if all(results.values()):
            print("  OK")
            passed_count += 1
        else:
            print("  FAILED")
            for k, v in results.items():
                if not v:
                    print(f"    - {k}")
                    
    print(f"\nSummary: {passed_count}/{len(files)} passed.")

if __name__ == '__main__':
    validate()
