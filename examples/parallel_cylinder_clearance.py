"""Evaluate the default cylinder clearance at saved inputs; no source edits.

python -m examples.parallel_cylinder_clearance --inputs design_optimized.json
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import openmdao
import openmdao.api as om
from rover.problem import build_problem
from reporting.case_inputs import read_case
from reporting.report import snapshot
from reporting.html_report import write_report


def evaluate(inputs, output):
    values,source=read_case(inputs)
    structure,motors=source.get('structure','reference'),source.get('motors','reference')
    output.mkdir(parents=True,exist_ok=True)
    p=build_problem(values,structure=structure,motors=motors)
    try:
        p.run_model()
        state=snapshot(p)
        path=output/'cylinders.json'
        record={'schema_version':2,'openmdao_version':openmdao.__version__,
                'model_metadata':p.model_metadata,'baseline':state,'final':state,
                'optimizer_success':None,'status':'baseline',
                'notice':'Fixed-input evaluation of default clearance; no hardware validation.',
                'report_context':{'name':f'{source.get("name","Saved design")} / cylinder clearance',
                                  'revision':source.get('revision','unspecified'),'source':source['source'],
                                  'source_sha256':source['sha256'],'input_file_names':sorted(values),
                                  'cli_override_names':[],'mode':'fixed_design',
                                  'generated_utc':datetime.now(timezone.utc).isoformat(timespec='seconds')}}
        path.write_text(json.dumps(record,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        om.n2(p,outfile=str(output/'cylinders_n2.html'),show_browser=False)
        write_report(record,path)
        summary={n:state['outputs'][n]['value'] for n in (
            'obstacle_drum_wheel_gap_bound','obstacle_drum_wheel_margin',
            'construction_points','dry_mass','payload_mass','run_energy')}
        summary['modeled_feasible']=state['feasible']
        print(json.dumps(summary,indent=2))
        return summary
    finally:
        p.cleanup()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs',type=Path,default=Path('design_optimized.json'))
    parser.add_argument('--output',type=Path,default=Path('results/parallel_cylinder_clearance'))
    args=parser.parse_args()
    evaluate(args.inputs,args.output)
