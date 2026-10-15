"""Self-contained study explorer and CSV export; no network or plotting runtime."""
import csv
from html import escape
import json
from pathlib import Path


def write_explorer(study, output):
    output = Path(output)
    if study.get('motors') == 'x44-standard':
        # Independent optimistic limit for the prescribed profile: use the
        # narrowest track and largest wheel, even before allowing for load.
        from rover.config import configuration
        from rover.mobility import route_profiles
        from rover.alternatives.x44_motor import CURVE
        import math
        design, _ = configuration(study['structure'], study['motors'])
        v = study['source_inputs']
        fastest_shape = route_profiles(design['track_width'][1], v['travel_speed'], v['acceleration'],
                                       v['turn_speed_fraction'], v['turn_accel_fraction'], 15)
        required = max(p[3] for p in fastest_shape if p[0] == 'straight')
        free = CURVE.free_rpm/v['drive_gear_ratio']*math.pi*design['wheel_diameter'][2]/60
        study['independent_speed_limit'] = {'minimum_one_way_time_s_no_load':15*required/free,
                                            'required_peak_at_15s_m_s':required, 'maximum_no_load_speed_m_s':free,
                                            'scope':'Fixed route profile, largest permitted wheel, minimum track, fixed X44 drive gearing. Driving loads can only tighten this bound.'}
    rows = []
    for source in study['rows']:
        if source['status'] == 'error' and not source.get('analysis'):
            # There is no usable final state. Report the actual errors instead
            # of borrowing geometry/score from a different design or scenario.
            folder = output/'cases'/source['id']
            diagnostics = ''.join('<h2>'+escape(a['start'])+'</h2><pre>'+escape(error or 'No final state saved.')+'</pre>'
                                  for a, error in zip(source['attempts'], source.get('errors', [])))
            (folder/'error.html').write_text(
                '<!doctype html><html lang="en"><meta charset="utf-8"><title>Study solver diagnostics</title>'
                '<style>body{font:15px/1.6 system-ui;max-width:1100px;margin:40px auto;padding:20px}pre{white-space:pre-wrap;overflow-wrap:anywhere}h1{color:#a13e25}</style>'
                f'<h1>No usable optimized design: {source["traversal_time_s"]:g} s / fill ceiling {source["fill_fraction"]:g}</h1>'
                '<p>Both starts ended without a usable final numeric state. No optimized geometry or valid optimum is claimed for this point. '
                'The model constraints and failure status have not been relaxed.</p><p><a href="../../index.html">Return to heat map</a></p>'+diagnostics+'</html>', encoding='utf-8')
            source['report'] = f"cases/{source['id']}/error.html"
        row = {k: v for k, v in source.items() if k not in ('assumptions', 'fingerprint')}
        if source.get('analysis'):
            stem = output/source['analysis']
            row['views'] = {pose: stem.with_name(stem.stem+'_'+pose+'.svg').read_text(encoding='utf-8')
                            for pose in ('top', 'travel', 'retracted', 'extended')}
        rows.append(row)
    payload = {k: v for k, v in study.items() if k not in ('rows', 'source_model_sha256', 'source_inputs')}
    payload['baseline_time_s'] = study['source_inputs']['traversal_time_one_way']
    payload['baseline_fill'] = study['source_inputs']['fill_fraction']
    payload['rows'] = rows
    template = Path(__file__).with_name('traversal_fill_explorer.html').read_text(encoding='utf-8')
    text = json.dumps(payload, ensure_ascii=True, allow_nan=False).replace('<', '\\u003c')
    temporary = output/'index.html.tmp'
    temporary.write_text(template.replace('__STUDY_DATA__', text), encoding='utf-8')
    temporary.replace(output/'index.html')
    design_names = sorted({n for row in rows for n in row.get('design', {})})
    names = ['id', 'traversal_time_one_way_s', 'fill_fraction', 'status', 'selected_start',
             'construction_points', 'mass_points', 'energy_points', 'whole_cycle_points', 'completed_cycles',
             'dry_mass_kg', 'payload_mass_kg', 'mission_energy_Wh', 'cycle_time_s', 'storage_use_fraction',
             'failed_checks', 'report', *design_names]
    with (output/'summary.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=names)
        writer.writeheader()
        for row in rows:
            outputs, whole = row.get('outputs', {}), row.get('whole_cycles', {})
            writer.writerow({'id': row['id'], 'traversal_time_one_way_s': row['traversal_time_s'],
                             'fill_fraction': row['fill_fraction'], 'status': row['status'],
                             'selected_start': row.get('selected_start'),
                             **{k: outputs.get(k) for k in ('construction_points', 'mass_points', 'energy_points')},
                             'whole_cycle_points': whole.get('construction_points'), 'completed_cycles': whole.get('completed'),
                             **{k: outputs.get(n) for k, n in (('dry_mass_kg', 'dry_mass'), ('payload_mass_kg', 'payload_mass'),
                                                               ('mission_energy_Wh', 'run_energy'), ('cycle_time_s', 'cycle_time'))},
                             'storage_use_fraction': row.get('storage_use_fraction'),
                             'failed_checks': '; '.join(row.get('failed_checks', {})), 'report': row.get('report'),
                             **row.get('design', {})})
    # Rendering can attach an error report or an independent hardware bound.
    # Keep those additions in the saved manifest on the initial run and resume.
    if study.get('fingerprint'):
        temporary = output/'study.json.tmp'
        temporary.write_text(json.dumps(study, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        temporary.replace(output/'study.json')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Regenerate a saved study explorer without rerunning optimizations.')
    parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    study = json.loads((args.folder/'study.json').read_text(encoding='utf-8'))
    write_explorer(study, args.folder)
    (args.folder/'study.json').write_text(json.dumps(study, indent=2, allow_nan=False)+'\n', encoding='utf-8')
