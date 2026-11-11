# Geometry and static-stability study — 2026-11-07

The selected candidate passes the geometry study with 359 mm wheels and
359 mm drums. No diameter reduction was needed after drum length was allowed
to vary. This is a bounded local design search, not proof of a global optimum
or a fully feasible mission design.

The student subsequently requested restoration of `design_baseline.json` to
its exact pre-search contents. The candidate described here is preserved in
`results/geometry_search/optimized_baseline_before_restore.json`; these results
do not describe the restored baseline. The model/optimizer improvements and
25 mm deposition requirement remain in place. The restored file's historical
actuator commands/allowance need reconciliation with the new discrete hardware
definitions before it can run.

## Selected result

| Quantity | Value |
| --- | --- |
| PA-04 stroke | 6 in, fixed hardware case |
| Installed command interval | 352.298–504.698 mm |
| Wheel diameter / drum diameter | 359 / 359 mm |
| Wheelbase / wheel-center track | 650 / 682.250 mm |
| Wheel width | 67 mm |
| Length of each drum | 120 mm |
| Predicted dry mass / full payload | 44.01 / 43.73 kg |
| Stationary asymmetric deposition edge distance | 135.97 mm |
| Required deposition distance / remaining reserve | 25 / 110.97 mm |
| Fully loaded pitch tipping/contact limit | 37.20 degrees |
| Fully loaded roll tipping/contact limit | 38.73 degrees |
| Worst-heading first wheel unloading | 33.57 degrees |
| Stowed length / width / height | 1498.50 / 749.25 / 741.43 mm |
| Maximum-extension axle height | -0.151 mm relative to ground |

The important trade is storage: shortening drums from 340 to 120 mm reduces
modeled full payload from 123.91 to 43.73 kg at the unchanged 0.9 fill fraction.
The wheels also narrow from the starting 152 to 67 mm. The existing contact-
pressure screen is retained and passes, but this wheel stiffness/contact-area
model is provisional. These dimensions require CAD and hardware verification.
Berm discharge reach and height are tradeoffs, not enforced 400 mm targets.

The plunge/recovery check remains the existing maximum-extension axle-at-ground
requirement, as explicitly confirmed by the student. This does not claim the
drum bottom can reach 450 mm below ground.

## Enforced geometry scope

The search retains mass and stowed/deployed envelopes, input bounds, compact/
travel/deposition/extension ordering, triangle/toggle validity, pivot mounting,
minimum commanded stroke, excavation operating reach/domain, axle ground reach,
ground clearance, wheel/drum swept clearance, all four raised-wheel finite-
cylinder collision cases, actuator force/curve domains, section validity and the
existing gravity structural stress/deflection/buckling/packing checks. Ground
pressure is screened at the configured acceleration upper bound using the same
contact-area/pressure helpers as Mobility; a narrow-wheel candidate that failed
that limit was rejected.

Static traversal keeps the full current compliant wheel/differential model,
nonnegative wheel reactions, numerical/branch validity, the 5-degree heading
grid with local refinement, and all 41 rock heights. Pitch and roll global-tip
and first-unloading angles are computed separately. Initial search evaluations
use selected exact equilibrium poses; a candidate is accepted only after the
complete existing heading/limit and rock screens pass. No coarser screen alone
establishes geometry feasibility. Arm pose angles parameterize search inputs;
saved cases retain the existing metre-valued actuator command ports.

Finite-cylinder wheel/drum clearance is also checked in the compliant poses
at each of the 41 heights for each of the four raised-wheel cases. The smallest
reserve beyond the existing 15 mm mechanism gap is 143.55 mm in those poses.

The maximum-stability pass maximizes a common static angle with both diameters
fixed. Both discrete actuator cases were considered. The selected 6-inch local
candidate has larger worst-heading stability than the tested 8-inch candidate.
No global optimality claim is made. Failed and intermediate cases remain under
`results/geometry_search/`; the prior baseline was saved before replacement.

## Remaining full-system failures

The complete model was evaluated after rounding the selected geometry, and its
full feasibility flag remains **false**. The remaining failures are:

- empty and loaded BWR climbing traction;
- digging/feed traction;
- turning traction;
- drum torque at the retained operating settings.

These checks were not weakened or called passed. In particular, static pitch/
roll stability is distinct from the ability to generate traction to climb that
slope. RPM, feed speed, gearbox selection, route and score semantics were not
optimized by this study. No soil, wheel or actuator measurements were invented.

## Reproduce

```powershell
.\.venv\Scripts\python.exe -m studies.geometry --inputs results/geometry_search/optimized_baseline_before_restore.json --stroke 6 --seconds 40 --maximize angle --output results/geometry_repeat.json
.\.venv\Scripts\python.exe main.py --inputs results/geometry_search/optimized_baseline_before_restore.json --baseline --report final --output results/geometry_candidate_repeat.json
```

Use `--stroke 8` for the other hardware case. `--vary-wheel --maximize wheel`
is an explicit fallback search when fixed-diameter searches do not find a
candidate; a failed bounded search is not proof of infeasibility. Search and
verification outputs preserve solver termination status separately from fixed
candidate geometry validity and full-system feasibility. The 25 mm deposition
minimum is a shared requirement in new normal runs, not just this study.

## Verification performed

61 focused tests passed across geometry requirements, PA-04 integration,
deposition states, finite-cylinder clearance, BWR mobility, calculated route
timing and the bounded development workflow. These include independent work/
energy and collision limits, invalid-domain rejection, dependency/N2 checks
and smooth derivative checks. The fast study's geometry adapters agree with
the full model; pressure screening is conservative relative to route outputs.

Both the reference baseline and selected baseline were evaluated. The selected
report and N2 diagram are `results/geometry_baseline.html` and
`results/geometry_baseline_n2.html`. At verification time the resolved rounded baseline inputs matched
the separately verified saved candidate exactly. A normal CLI total-derivative
check passed for wheel diameter/width; its optimization was deliberately stopped
at two evaluations and remains unsuccessful. The full test suite was not
completed; these are targeted checks, not a full-suite pass or experimental
validation.
