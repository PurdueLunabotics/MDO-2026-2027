"""Research-only IPEx horizontal-force surface; no rover discipline is replaced.

Run: python -m studies.ipex_force_surface --output results/ipex_force_surface
Fit the versioned Figure 13 trend readings offline, leave out complete depth
series for cross-validation, and explicitly flag extrapolated benchmark points.
"""
from dataclasses import asdict, dataclass
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data/excavation/ipex_force_v1"


def load_data(folder=DATA):
    """Read and verify processed data outside any model compute() call."""
    folder = Path(folder)
    manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
    if manifest.get("schema_version") != 1 or manifest.get("data_kind") != "digitized_published_trend":
        raise ValueError("Expected versioned digitized published trends, not raw experiments")
    filename = folder / "figure13_trends.csv"
    if hashlib.sha256(filename.read_bytes()).hexdigest() != manifest["files"][filename.name]:
        raise ValueError("Processed data hash mismatch")
    if manifest["units"] != {"depth_m": "m", "feed_speed_m_s": "m/s", "force_per_width_N_m": "N/m"}:
        raise ValueError("Unexpected data units")
    with filename.open(newline="", encoding="utf-8") as stream:
        rows = np.array([[float(r[n]) for n in ("depth_m", "feed_speed_m_s", "force_per_width_N_m")]
                         for r in csv.DictReader(stream)])
    return rows, manifest


def fit_surface(rows):
    """Offline least squares: F_x / width = depth * (a + b * feed_speed).

    Coefficients have units Pa and Pa*s/m. The zero-depth limit is imposed;
    nonnegative depth/speed sensitivities are checked, never obtained by clipping.
    """
    rows = np.asarray(rows, dtype=float)
    if rows.ndim != 2 or rows.shape[1] != 3 or len(rows) < 2 or not np.isfinite(rows).all() or np.any(rows <= 0):
        raise ValueError("Need finite positive depth, speed and force-per-width rows")
    design = np.column_stack((rows[:, 0], rows[:, 0] * rows[:, 1]))
    coefficients, _, rank, _ = np.linalg.lstsq(design, rows[:, 2], rcond=None)
    if rank != 2 or np.any(coefficients < 0):
        raise ValueError("Data do not support the selected monotone two-coefficient surface")
    return tuple(float(x) for x in coefficients)


def normalized_force(depth_m, feed_speed_m_s, coefficients):
    """Pure polynomial [N/m]; callers must separately assess source support."""
    a, b = coefficients
    if not all(math.isfinite(x) for x in (depth_m, feed_speed_m_s, a, b)) or min(depth_m, feed_speed_m_s, a, b) < 0:
        raise ValueError("Polynomial arguments must be finite and nonnegative")
    return depth_m * (a + b * feed_speed_m_s)


@dataclass(frozen=True)
class ForceEstimate:
    force_per_width_N_m: float
    force_per_drum_N: float
    four_drum_force_magnitude_sum_N: float
    tip_speed_to_feed_speed: float
    source_envelope_violations: tuple
    regime: str
    validation_status: str = "research_only; no independent hardware validation"


@dataclass(frozen=True)
class IpexForceSurface:
    coefficients: tuple
    envelope: dict

    def evaluate(self, *, depth_m, feed_speed_m_s, drum_width_m, drum_diameter_m,
                 rpm, allow_extrapolation=False):
        """Predict horizontal force for one drum, retaining all support failures.

        RPM is a support check, not an independently fitted force variable.
        Explicit extrapolation returns diagnostics; it never implies feasibility.
        The four-drum sum is a sum of magnitudes, not a net chassis reaction.
        """
        numbers = (depth_m, feed_speed_m_s, drum_width_m, drum_diameter_m, rpm)
        if not all(math.isfinite(x) for x in numbers) or depth_m < 0 or min(numbers[1:]) <= 0:
            raise ValueError("Need finite nonnegative depth and positive speed, dimensions and RPM")
        failures = []
        values = dict(depth_m=depth_m, feed_speed_m_s=feed_speed_m_s,
                      drum_width_m=drum_width_m, drum_diameter_m=drum_diameter_m,
                      width_to_diameter_ratio=drum_width_m/drum_diameter_m)
        for name, value in values.items():
            lo, hi = self.envelope[name]
            if value < lo - 1e-12:
                failures.append(name + "_below_source_range")
            if value > hi + 1e-12:
                failures.append(name + "_above_source_range")
        ratio = math.pi * drum_diameter_m * rpm / (60 * feed_speed_m_s)
        reference = self.envelope["tip_speed_to_feed_speed"]
        if abs(ratio/reference - 1) > self.envelope["tip_ratio_relative_tolerance"]:
            failures.append("tip_feed_ratio_outside_source_condition; independent_RPM_effect_unidentified")
        if failures and not allow_extrapolation:
            raise ValueError("Outside source support: " + "; ".join(failures))
        force = normalized_force(depth_m, feed_speed_m_s, self.coefficients)
        return ForceEstimate(force, force * drum_width_m, 4 * force * drum_width_m,
                             ratio, tuple(failures), "extrapolated" if failures else "source_envelope_only")


def error_metrics(observed, predicted):
    observed, predicted = np.asarray(observed), np.asarray(predicted)
    residual = predicted - observed
    return dict(mae_N_m=float(np.abs(residual).mean()), rmse_N_m=float(np.sqrt(np.mean(residual**2))),
                max_absolute_error_N_m=float(np.max(np.abs(residual))),
                max_relative_error=float(np.max(np.abs(residual/observed))))


def validate_surface(rows):
    """Hold out both speeds of one complete depth curve at a time.

    This tests depth transfer between published curves. Those curves are already
    regressions of NASA measurements; it is not independent hardware validation.
    """
    rows = np.asarray(rows, dtype=float)
    coefficients = fit_surface(rows)
    predictions = [normalized_force(d, v, coefficients) for d, v, _ in rows]
    held_out, folds = [], []
    for depth in sorted(set(rows[:, 0])):
        train = rows[:, 0] != depth
        model = fit_surface(rows[train])
        test = rows[~train]
        predicted = [normalized_force(d, v, model) for d, v, _ in test]
        held_out.extend(zip(test[:, 2], predicted))
        folds.append(dict(held_out_depth_m=float(depth), training_depths_m=sorted(set(rows[train, 0])),
                          observed_N_m=test[:, 2].tolist(), predicted_N_m=predicted,
                          metrics=error_metrics(test[:, 2], predicted)))
    observed, predicted = zip(*held_out)
    return dict(in_sample_curve_approximation=error_metrics(rows[:, 2], predictions),
                leave_one_depth_out=error_metrics(observed, predicted), folds=folds,
                independent_hardware_validation="unavailable")


def plot_surface(rows, surface, benchmark, output):
    """Standalone scientific plot; no browser or simulator required."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(13, 5.8))
    ax = fig.add_subplot(121, projection="3d")
    depth, speed = np.meshgrid(np.linspace(10, 25, 35), np.linspace(10, 30, 35))
    a, b = surface.coefficients
    force = depth/1000 * (a + b*speed/1000)
    ax.plot_surface(depth, speed, force, cmap="viridis", alpha=.72)
    ax.scatter(rows[:, 0]*1000, rows[:, 1]*1000, rows[:, 2], c="#172d36", s=35, depthshade=False)
    ax.set(xlabel="Cut depth (mm)", ylabel="Feed speed (mm/s)", zlabel="Horizontal force / width (N/m)",
           title="Surface within published coordinate bounds")
    ax.view_init(elev=24, azim=-128)
    ax2 = fig.add_subplot(122)
    depth_m = benchmark["inputs"]["depth_m"]
    speeds = np.linspace(0, max(.21, benchmark["inputs"]["feed_speed_m_s"]*1.05), 300)
    forces = depth_m * (a + b*speeds)
    inside = (speeds >= .01) & (speeds <= .03)
    ax2.axvspan(10, 30, color="#dcece5", alpha=.8, label="Published speed range")
    ax2.plot(speeds*1000, np.where(inside, forces, np.nan), color="#176a67", lw=2.3, label="Fitted surface")
    ax2.plot(speeds*1000, np.where(~inside, forces, np.nan), "--", color="#ba6d22", lw=2, label="Polynomial extrapolation")
    selected = rows[:, 0] == depth_m
    ax2.scatter(rows[selected, 1]*1000, rows[selected, 2], color="#172d36", zorder=5, label="Published trend readings")
    prediction = benchmark["empirical"]
    x = benchmark["inputs"]["feed_speed_m_s"]*1000
    ax2.scatter([x], [prediction["force_per_width_N_m"]], marker="*", s=160, color="#ba6d22", zorder=6,
                label="Requested benchmark (unsupported)")
    ax2.scatter([x], [benchmark["reference_pressure_law_force_per_width_N_m"]], marker="x", s=80,
                color="#3859a1", label="Current pressure-law benchmark")
    ax2.set(xlabel="Feed speed (mm/s)", ylabel="Horizontal force / width (N/m)",
            title=f"{depth_m*1000:g} mm depth: tested range and benchmark", xlim=(0, speeds[-1]*1000), ylim=(0, None))
    ax2.grid(alpha=.25)
    ax2.legend(fontsize=8, loc="upper left")
    fig.suptitle("IPEx Figure 13 research surface - digitized published trends", fontsize=14)
    fig.text(.5, .025, "NASA tip/feed ratio = 8.5. No independent RPM fit, raw-run uncertainty, or hardware validation.\n"
             "Figure/table depth-label inconsistency remains unresolved. Horizontal force does not identify shaft torque.",
             ha="center", fontsize=9, color="#536579")
    fig.subplots_adjust(left=.055, right=.97, top=.85, bottom=.21, wspace=.32)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, default=ROOT / "design_baseline.json")
    parser.add_argument("--output", type=Path, default=ROOT / "results/ipex_force_surface")
    args = parser.parse_args()
    rows, manifest = load_data()
    surface = IpexForceSurface(fit_surface(rows), manifest["source_envelope"])
    from rover.config import configuration
    from reporting.case_inputs import read_case
    from rover.alternatives.moving_excavation import moving_supply
    supplied, source = read_case(args.inputs)
    design, assumptions = configuration()
    values = {n: spec[0] for group in (design, assumptions) for n, spec in group.items()}
    values.update(supplied)
    request = dict(depth_m=values["inlet_depth"], feed_speed_m_s=values["feed_speed"],
                   drum_width_m=values["drum_length"], drum_diameter_m=values["drum_diameter"], rpm=values["drum_rpm"])
    prediction = surface.evaluate(**request, allow_extrapolation=True)
    supply = moving_supply(values["drum_diameter"]/2, values["drum_length"], values["inlet_depth"],
                           values["fill_fraction"], values["usable_volume_fraction"], values["stored_soil_density"],
                           values["soil_density"], values["capture_fraction"], values["feed_speed"],
                           values["drum_rpm"], values["scoops_per_revolution"])
    old_force = values["cut_pressure"] * values["drum_length"] * supply["feed_per_scoop"]
    benchmark = dict(inputs=request, empirical=asdict(prediction), reference_pressure_law_force_per_drum_N=old_force,
                     reference_pressure_law_force_per_width_N_m=old_force/values["drum_length"],
                     reference_pressure_law_four_drum_force_magnitude_sum_N=4*old_force,
                     empirical_to_reference_ratio=prediction.force_per_drum_N/old_force)
    result = dict(revision=manifest["revision"], data_kind=manifest["data_kind"],
                  data_sha256=manifest["files"]["figure13_trends.csv"], source=manifest["source"],
                  study_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  manifest_sha256=hashlib.sha256((DATA / "manifest.json").read_bytes()).hexdigest(),
                  formula="F_x / width = depth_m * (a_Pa + b_Pa_s_m * feed_speed_m_s)",
                  coefficients=dict(a_Pa=surface.coefficients[0], b_Pa_s_m=surface.coefficients[1]),
                  validation=validate_surface(rows), benchmark=benchmark,
                  baseline_source_sha256=source["sha256"], limitations=manifest["limitations"],
                  integration="Study only. No ModelSet, compute(), defaults, feasibility, torque or mission outputs changed.")
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "surface.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    plot_surface(rows, surface, benchmark, args.output / "surface.png")
    print(json.dumps(dict(coefficients=result["coefficients"], validation=result["validation"]["leave_one_depth_out"],
                          benchmark=benchmark, output=str(args.output.resolve())), indent=2))


if __name__ == "__main__":
    main()
