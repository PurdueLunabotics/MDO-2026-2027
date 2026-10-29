"""Optimize a Cartesian grid of one-way route time and drum storage ceilings.

python -m studies.traversal_fill --workers 6
Outputs stay under ignored results/. This never rewrites either design JSON.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
import math
import os
from pathlib import Path
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TIMES = tuple(range(15, 181, 15))
DEFAULT_FILLS = tuple(i/10 for i in range(2, 10))


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    temporary.replace(path)


def validate_grid(times, fills):
    if not times or not fills or len(set(times)) != len(times) or len(set(fills)) != len(fills):
        raise ValueError("Grid axes must be nonempty and have no duplicate values.")
    if any(isinstance(t, bool) or not math.isfinite(t) or not 15 <= t <= 180 for t in times):
        raise ValueError("Traversal times must be finite and between 15 and 180 seconds one way.")
    if any(isinstance(f, bool) or not math.isfinite(f) or not .2 <= f <= .9 for f in fills):
        raise ValueError("Fill ceilings must be finite and between 0.2 and 0.9.")
    return sorted(times), sorted(fills)


def case_id(seconds, fill):
    def token(value):
        return f"{value:g}".replace('.', 'p')
    return f"t{token(seconds)}_f{token(fill)}"


def violation(state, metadata):
    """Rank failed diagnostics in dimensionless units, never as valid optima."""
    from rover.config import REQUIREMENTS
    specs = metadata['configuration']
    total = 0.
    for name, check in state['constraints'].items():
        if not check['passed']:
            if name in REQUIREMENTS:
                scale = REQUIREMENTS[name][0]
            elif name.endswith('_bounds'):
                spec = specs['design'][name[:-7]]
                scale = spec[2]-spec[1]
            else:
                scale = specs['margins'][name][0]
            total += max(0., -check['margin'])/scale
    return total


def select_attempt(attempts):
    eligible = [r for r in attempts if r.get('optimizer_success') and r.get('final', {}).get('feasible')]
    score = lambda r: r['final']['outputs']['construction_points']['value']
    if eligible:
        return max(eligible, key=score)
    diagnostics = [r for r in attempts if r.get('final')]
    return min(diagnostics, key=lambda r: (violation(r['final'], r['model_metadata']), -score(r))) if diagnostics else None


def run_attempt(job, seed_name, seed_design):
    from rover.problem import build_problem
    from reporting.report import snapshot
    import openmdao
    values = {**job['source_inputs'], **seed_design,
              'traversal_time_one_way': job['seconds'], 'fill_fraction': job['fill']}
    p = None
    started = time.perf_counter()
    record = {'schema_version': 3, 'openmdao_version': openmdao.__version__,
              'optimizer_success': False, 'status': 'error', 'notice': 'Uncalibrated conceptual estimates.',
              'report_context': {'name': f"Traversal {job['seconds']:g} s / fill ceiling {job['fill']:.2f}",
                                 'revision': job['source_revision'], 'source': job['source_name'],
                                 'source_sha256': job['source_sha256'], 'generated_utc': utc_now(),
                                 'mode': 'optimization', 'input_file_names': sorted(job['source_inputs']),
                                 'cli_override_names': ['traversal_time_one_way', 'fill_fraction'],
                                 'source_notes': {'study': 'traversal_fill', 'start': seed_name,
                                                  'study_fingerprint': job['fingerprint']}}}
    try:
        p = build_problem(values, structure=job['structure'], motors=job['motors'], traversal_mode="fixed",
                          actuators=job.get('actuators', 'reference'),
                          maxiter=job['maxiter'], display=False)
        record['model_metadata'] = p.model_metadata
        assert not {'traversal_time_one_way', 'fill_fraction'} & set(p.design_specs)
        p.run_model()
        record['baseline'] = snapshot(p)
        result = p.run_driver()
        p.run_model()
        record['final'] = snapshot(p)
        record['optimizer_success'] = bool(result.success)
        record['optimizer_result'] = str(result)
        record['optimizer'] = p.model_metadata['optimizer_settings']['optimizer']
        record['status'] = 'converged_feasible' if result.success and record['final']['feasible'] else 'failed_or_infeasible'
        for name, requested in (('traversal_time_one_way', job['seconds']), ('fill_fraction', job['fill'])):
            if p.get_val(name).item() != requested:
                raise RuntimeError(f"The swept input {name} was not held fixed.")
    except Exception:
        record.update(status='error', optimizer_success=False, error=traceback.format_exc())
        # A computation exception cannot promote its last diagnostic state.
        record.pop('final', None)
    finally:
        if p is not None:
            p.cleanup()
    record['study_attempt'] = {'start': seed_name, 'wall_seconds': time.perf_counter()-started,
                               'requested_time_s': job['seconds'], 'requested_fill_fraction': job['fill']}
    return record


def run_cell(job):
    """One independent process task, with serial, deterministic starts."""
    folder = Path(job['output'])/'cases'/case_id(job['seconds'], job['fill'])
    folder.mkdir(parents=True, exist_ok=True)
    attempts = []
    with (folder/'solver.log').open('w', encoding='utf-8') as log, redirect_stdout(log), redirect_stderr(log):
        for seed_name, seed_design in job['starts']:
            print(f"Start: {seed_name}", flush=True)
            result = run_attempt(job, seed_name, seed_design)
            atomic_json(folder/f"attempt_{seed_name}.json", result)
            attempts.append(result)
        selected = select_attempt(attempts)
        if selected:
            atomic_json(folder/'analysis.json', selected)
            from reporting.html_report import write_report
            # Detailed reports retain actual start/final states and saved failures.
            write_report(selected, folder/'analysis.json', download_links={'inputs': None, 'n2': None})
    row = {'id': folder.name, 'traversal_time_s': job['seconds'], 'fill_fraction': job['fill'],
           'fingerprint': job['fingerprint'], 'status': selected['status'] if selected else 'error',
           'report': f"cases/{folder.name}/analysis.html" if selected else None,
           'analysis': f"cases/{folder.name}/analysis.json" if selected else None,
           'selected_start': selected['study_attempt']['start'] if selected else None,
           'attempts': [{'start': r['study_attempt']['start'], 'status': r['status'],
                         'optimizer_success': r['optimizer_success'],
                         'feasible': r.get('final', {}).get('feasible', False),
                         'construction_points': r.get('final', {}).get('outputs', {}).get('construction_points', {}).get('value'),
                         'wall_seconds': r['study_attempt']['wall_seconds']} for r in attempts]}
    if selected:
        state = selected['final']
        row['optimizer_success'], row['feasible'] = selected['optimizer_success'], state['feasible']
        row['outputs'] = {n: i['value'] for n, i in state['outputs'].items()}
        row['design'] = {n: i['value'] for n, i in state['design'].items()}
        row['whole_cycles'] = state['whole_cycles']
        row['failed_checks'] = {n: c for n, c in state['constraints'].items() if not c['passed']}
        row['assumptions'] = {n: i['value'] for n, i in state['assumptions'].items()}
        row['storage_use_fraction'] = row['outputs']['payload_mass']/row['outputs']['target_payload_mass']
    else:
        row['errors'] = [r.get('error') for r in attempts]
    atomic_json(folder/'cell.json', row)
    return row


def load_study_inputs(path, seed_path):
    from reporting.case_inputs import read_case
    from rover.config import configuration
    source, meta = read_case(path)
    structure, motors = meta.get('structure', 'reference'), meta.get('motors', 'reference')
    design, assumptions = configuration(structure, motors, meta.get('actuators', 'reference'))
    # Freeze all effective assumptions for reproducibility even for a partial source file.
    complete = {n: spec[0] for n, spec in {**design, **assumptions}.items()}
    complete.update(source)
    starts = [('baseline', {})]
    seeds = {}
    if seed_path:
        candidate, seed_meta = read_case(seed_path)
        if (seed_meta.get('structure', structure) != structure or seed_meta.get('motors', motors) != motors
                or seed_meta.get('actuators', 'reference') != meta.get('actuators', 'reference')):
            raise ValueError('The second starting design must use the baseline hardware profiles.')
        seeds = {n: candidate[n] for n in design if n in candidate}
        starts.append(('saved_candidate', seeds))
    return complete, meta, structure, motors, starts


def build_study(args):
    times, fills = validate_grid(args.times, args.fills)
    values, meta, structure, motors, starts = load_study_inputs(args.inputs, args.seed)
    actuators = meta.get('actuators', 'reference')
    files = sorted((ROOT/'rover').rglob('*.py')) + sorted((ROOT/'data/motors').glob('*.csv')) + [ROOT/'data/motors/manifest.json']
    if actuators == 'pa04-900':
        files += sorted((ROOT/'data/actuators').glob('*.csv')) + [ROOT/'data/actuators/manifest.json']
    model_hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    versions = {name: version(name) for name in ('openmdao', 'numpy', 'scipy')}
    fingerprint = digest({'source_inputs': values, 'source_sha256': meta['sha256'], 'starts': starts,
                          'structure': structure, 'motors': motors, 'actuators': actuators, 'maxiter': args.maxiter,
                          'model_hashes': model_hashes, 'versions': versions,
                          'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()})
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = {'schema_version': 1, 'created_utc': utc_now(), 'source': args.inputs.name,
                'source_sha256': meta['sha256'], 'source_inputs': values, 'source_model_sha256': model_hashes,
                'fingerprint': fingerprint, 'structure': structure, 'motors': motors, 'actuators': actuators,
                'traversal_times_s': times, 'fill_fractions': fills, 'starts': [n for n, _ in starts],
                'maxiter_per_start': args.maxiter, 'package_versions': versions,
                'objective': 'maximum saved continuous construction_points among converged, modeled-feasible starts',
                'limitations': 'Best local result from deterministic starts, not a proven global optimum. Uncalibrated physics. Fill is a storage ceiling; actual payload can be soil-reach limited.',
                'rows': []}
    jobs, rows = [], []
    for fill in fills:
        for seconds in times:
            cached = output/'cases'/case_id(seconds, fill)/'cell.json'
            if cached.exists():
                row = json.loads(cached.read_text(encoding='utf-8'))
                if row.get('fingerprint') != fingerprint:
                    raise ValueError(f"Stale case {cached}; choose a new output folder for changed inputs/settings/code.")
                rows.append(row)
            else:
                jobs.append({'seconds': seconds, 'fill': fill, 'output': str(output), 'source_inputs': values,
                             'source_name': args.inputs.name, 'source_revision': meta.get('revision', 'Not specified'),
                             'source_sha256': meta['sha256'], 'structure': structure, 'motors': motors, 'actuators': actuators,
                             'maxiter': args.maxiter, 'starts': starts, 'fingerprint': fingerprint})
    from .traversal_fill_view import write_explorer
    def save_progress():
        manifest['rows'] = sorted(rows, key=lambda r: (r['fill_fraction'], r['traversal_time_s']))
        manifest['completed_cases'] = len(rows)
        manifest['converged_feasible_cases'] = sum(r['status'] == 'converged_feasible' for r in rows)
        atomic_json(output/'study.json', manifest)
        write_explorer(manifest, output)
    save_progress()
    print(f"{len(times)} x {len(fills)} = {len(times)*len(fills)} cases, {len(starts)} starts/case; {len(rows)} cached.", flush=True)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(run_cell, job): job for job in jobs}
        for future in as_completed(futures):
            row = future.result()
            rows.append(row)
            save_progress()
            print(f"[{len(rows)}/{len(times)*len(fills)}] {row['id']}: {row['status']} "
                  f"{row.get('outputs', {}).get('construction_points', float('nan')):.2f} points", flush=True)
    print(f"Saved {output/'index.html'}", flush=True)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', type=Path, default=Path('design_baseline.json'))
    parser.add_argument('--seed', type=Path, help='Optional second start with matching hardware profiles; historical candidates must be migrated explicitly')
    parser.add_argument('--times', nargs='+', type=float, default=DEFAULT_TIMES)
    parser.add_argument('--fills', nargs='+', type=float, default=DEFAULT_FILLS)
    parser.add_argument('--workers', type=int, default=6)
    parser.add_argument('--maxiter', type=int, default=2000)
    parser.add_argument('--output', type=Path, default=Path('results/sensitivity/traversal_fill'))
    args = parser.parse_args()
    if args.workers < 1 or args.maxiter < 1:
        parser.error('Workers and maxiter must be positive.')
    # Each worker owns its model; avoid multiplying BLAS thread pools per worker.
    for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
        os.environ[name] = '1'
    build_study(args)


if __name__ == '__main__':
    main()
