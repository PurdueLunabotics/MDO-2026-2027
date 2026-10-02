"""Validated 1-D response table for a FIXED-condition test slice.

This is a teaching adapter, not permission to extrapolate across drum designs.
Synthetic fixtures require explicit opt-in; out-of-domain requests raise rather
than silently clipping. A future multidimensional adapter must retain these rules.
"""
import json
from pathlib import Path
import numpy as np


class ResponseTable:
    def __init__(self, path, *, allow_synthetic=False):
        self.path = Path(path)
        self.data = json.loads(self.path.read_text(encoding="utf-8"))
        d = self.data
        if d.get("schema_version") != 1 or d.get("response") != "granular_response":
            raise ValueError("Unsupported response-table schema/type")
        if d.get("data_kind") not in {"synthetic", "experiment", "edem"}:
            raise ValueError("Record data_kind: synthetic, experiment, or edem")
        if d["data_kind"] == "synthetic" and not allow_synthetic:
            raise ValueError("Synthetic data are for adapter tests only; explicit opt-in required")
        for key in ("source", "revision", "material", "conditions", "validation"):
            if not d.get(key):
                raise ValueError(f"Missing provenance field: {key}")
        if d.get("units") != {"froude": "1", "capture": "1", "lift": "1"}:
            raise ValueError("Expected dimensionless froude/capture/lift columns")
        self.rows = np.asarray(d["rows"], dtype=float)
        if self.rows.ndim != 2 or self.rows.shape[1] != 3 or len(self.rows)<2 or not np.isfinite(self.rows).all():
            raise ValueError("rows must be finite [Froude, capture, effective_lift] triples")
        if np.any(np.diff(self.rows[:,0])<=0) or self.rows[0,0]<0:
            raise ValueError("Froude values must be nonnegative and strictly increasing")
        if np.any((self.rows[:,1]<=0)|(self.rows[:,1]>1)) or np.any(self.rows[:,2]<0):
            raise ValueError("Require 0<capture<=1 and lift>=0")
        for name,spec in d["conditions"].items():
            if set(spec)!={"value","units","absolute_tolerance"}:
                raise ValueError(f"Malformed fixed condition {name}")
            if not np.isfinite([spec["value"],spec["absolute_tolerance"]]).all() or spec["absolute_tolerance"]<0:
                raise ValueError(f"Invalid condition tolerance {name}")

    def evaluate(self, froude, conditions):
        if not np.isfinite(froude) or not self.rows[0,0] <= froude <= self.rows[-1,0]:
            raise ValueError(f"Froude {froude} outside validated table [{self.rows[0,0]}, {self.rows[-1,0]}]")
        for name,spec in self.data["conditions"].items():
            if name not in conditions or not np.isfinite(conditions[name]) or abs(conditions[name]-spec["value"])>spec["absolute_tolerance"]:
                raise ValueError(f"{name} outside fixed-condition test slice; collect data before changing geometry/soil")
        return tuple(float(np.interp(froude,self.rows[:,0],self.rows[:,j])) for j in (1,2))
