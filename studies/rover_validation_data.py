"""Versioned measured rover points and radius conventions; no prediction model."""
import hashlib
import json
from pathlib import Path

DATA = Path(__file__).resolve().parents[1] / "data/mobility/external_validation/large_rovers_2026_10_23.json"


def measurements():
    manifest = json.loads(DATA.read_text(encoding="utf-8"))
    rows = []
    for case in manifest["cases"]:
        points = case.get("observations", [])
        if "digitization" in case:
            spec = case["digitization"]
            x0, x1 = spec["slope_axis_px"]
            a0, a1 = spec["slope_axis_deg"]
            y0, y1 = spec["slip_axis_px"]
            s0, s1 = spec["slip_axis_values"]
            points = [{"slope_deg": a0+(x-x0)/(x1-x0)*(a1-a0),
                       "measured_slip": s0+(y-y0)/(y1-y0)*(s1-s0), "marker_xy_px": [x, y]}
                      for x, y in spec["marker_xy_px"]]
        for point in points:
            rows.append({"case": case["id"], "label": case["label"], "mass_kg": case["mass_kg"],
                         "wheel_count": case["wheel_count"], "diameter_m": case["diameter_m"],
                         "width_m": case["width_m"], **point,
                         "measured_outer_reference_slip": 1-(1-point["measured_slip"])*case["slip_reference_diameter_m"]/case["diameter_m"]})
    return {"manifest": manifest, "manifest_sha256": hashlib.sha256(DATA.read_bytes()).hexdigest(), "results": rows}
