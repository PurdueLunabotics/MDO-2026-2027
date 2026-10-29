"""Offline affine mass fit, held-out catalog errors, and source hashes.

Run: .venv/Scripts/python.exe data/actuators/fit_pa04.py
Validation checks interpolation of the supplied catalog, not actual hardware.
"""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def fit():
    root = Path(__file__).parent
    with (root/'pa04_mass.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    train = [r for r in rows if r['split'] == 'fit']
    matrix = np.array([[1., float(r['stroke_in'])] for r in train])
    intercept, slope = np.linalg.lstsq(matrix, [float(r['mass_lb']) for r in train], rcond=None)[0]
    errors = {}
    for split in ('fit', 'validation', 'all'):
        selected = [r for r in rows if split == 'all' or r['split'] == split]
        residual = np.array([intercept+slope*float(r['stroke_in'])-float(r['mass_lb']) for r in selected])
        errors[split] = {'count': len(selected), 'mae_lb': float(np.mean(abs(residual))),
                         'max_abs_lb': float(max(abs(residual))), 'rmse_lb': float(np.sqrt(np.mean(residual**2)))}
    return {'intercept_lb': float(intercept), 'slope_lb_per_in': float(slope), 'errors': errors,
            'sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.glob('*.csv'))}}


if __name__ == '__main__':
    print(json.dumps(fit(), indent=2))
