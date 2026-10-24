# Default rigid-wheel Bekker-Wong-Reece model

Revision `bwr-grc1-oravec164-v2` is the default straight-driving model and supplies
the new empty/full climb checks. The student selected rigidity as a conservative
design assumption. GRC-1 parameters remain published proxies without rover
refitting or prototype calibration; the failed independent accuracy gate is
retained below. The earlier straight-slip surrogate has been deleted.

## Equations and conventions

The pure functions live in `rover/alternatives/bekker_wong_reece.py`. Wheel radius
`r`, width `b`, sinkage `z` and shear displacement `j` are metres, forces are N,
torque is N*m, stresses are Pa, angles are radians, and slip is dimensionless.
Each wheel calculation is for ONE wheel. Whole-rover mass includes the actual
loading state; there is no change to wheel/drum mass accounting.

The contact arc is measured from the downward plane-normal, positive forward.
The exit angle is zero, an explicit assumption of no elastic soil rebound.

```text
theta_f = acos(1-z/r)
theta_m = (c1+c2*s)*theta_f
q = kc/b+kphi
front sigma(theta) = q*[r*(cos(theta)-cos(theta_f))]^n
rear  psi(theta) = theta_f-theta/theta_m*(theta_f-theta_m)
rear  sigma(theta) = q*[r*(cos(psi)-cos(theta_f))]^n
j(theta) = r*[(theta_f-theta)-(1-s)*(sin(theta_f)-sin(theta))]
tau(theta) = [c+sigma(theta)*tan(phi)]*[1-exp(-j(theta)/K)]
```

The rear distribution is continuous at the maximum and zero at the exit; both
normal-stress endpoints are zero. The implemented rear equation follows Zhou
et al. (2014), equation 6. The rendered rear-penetration expression in the NASA
overview has a typographical problem; it is not copied into executable physics.

All integrals run from zero to `theta_f`, split at `theta_m` with 64-point
Gauss-Legendre quadrature on each interval:

```text
N = b*r*integral(sigma*cos(theta)+tau*sin(theta))
thrust = b*r*integral(tau*cos(theta))
compaction_resistance = b*r*integral(sigma*sin(theta))
Fx = thrust-compaction_resistance
T = b*r^2*integral(tau)
```

At prescribed slip, a bracketed scalar solve sets `N=normal_load`. For drawbar
demand, 33 samples over `0<=s<=1` bracket the first resolved force crossing;
sampled local maxima are refined when no sampled crossing exists. This is not
a proof against arbitrary narrow branches in an untested soil law. Operating
solutions require `s<1`; `s=1` is evaluated only as a locked-wheel force limit.
`z/r<=0.95` bounds the contact geometry. Unsupported contact inputs still raise
`WheelDomainError`; unreachable drive demands raise `NoDrivingSolution` carrying
the resolved limiting force. No failed solve is converted into a valid prediction.

For a uniform uphill plane, equal average wheel loads are assumed:

```text
N_each = mass*g*cos(alpha)/wheel_count
Fx_required_each = mass*(g*sin(alpha)+acceleration)/wheel_count
```

This resolves gravity along/normal to the plane but has no separately calibrated
slope-induced weakening, soil flow, front/rear multipass, axle load transfer or
torque allocation. The empty/full BWR checks now enforce the student's slip-only climbing rule. The existing 30-degree static stability requirement
is unchanged and remains a separate test.

## Independent soil data and validity

The versioned manifest `data/mobility/bekker_wong_reece/grc1_loose.json` includes
source URLs, PDF hashes, table locations, SI conversions, assumptions, and the
accuracy gate specified before the first BWR benchmark. No rover observations
are used to fit any coefficient.

Oravec (2009), Tables 5.4 and 5.5, GRC-1 at 1.64 g/cc / 14% relative density:

| Parameter | Value | Meaning |
| --- | --- | --- |
| `n` | 0.95 | pressure-sinkage exponent |
| `kc` | 221260 N/m^(n+1) | cohesive pressure-sinkage modulus |
| `kphi` | 1957510 N/m^(n+2) | frictional pressure-sinkage modulus |
| `c` | 10 Pa | measured apparent cohesion |
| `phi` | 33.3 degrees | shear friction angle |
| `K` | 0.0255 m | shear displacement modulus; published SD 0.0092 m |

The rounded Table 5.4 values are used consistently, without mixing rounded `n`
with Figure 5.57's higher-precision moduli. The fitted plate widths are
76, 102 and 190 mm. Figures cover pressure to 50 kPa and sinkage to 10 mm;
shear tests cover normal stresses of 5-30 kPa. These are plate/annulus tests,
not a validated wheel envelope. The pure solver reports stresses and sinkage;
the offline report flags excursions beyond the plotted windows. Values inside
the windows still require plate-to-wheel transfer.

`c1=0.18,c2=0.32` come from the generic loose-sand row in NASA/TM-20250006958,
Table 2. They are **assumptions for this GRC-1**, not measured GRC-1 stress-peak
parameters. Denser GRC-1 shear rows contain negative apparent cohesion fits,
and the 1.75 g/cc pressure coefficients differ between a table and a figure.
No negative fit is silently clipped and no alternative density set is invented.

The preset is restricted to Earth gravity. The solver is quasi-static and does
not establish a speed range. The source laboratory fits cannot validate the
rover's higher speed. No compliance, explicit grousers/tread, bulldozing,
velocity dependence, spin inertia or soil-strength changes are modeled.

## Larger-rover benchmark

```powershell
.\.venv\Scripts\python.exe -m studies.bekker_wong_reece
```

Open `results/bekker_wong_reece/index.html` for measured/predicted curves,
failures and sensitivity; `comparison.json` preserves all values. The external
measurements and geometry provenance are versioned in the earlier larger-rover
manifest. The Husky soil was prepared at 28-31% relative density, so the 14%
preset comparison tests preparation transfer, not matched-soil validation.

Stock Husky, 63 kg, assumed 330 x 125 mm tires, rigid-cylinder proxy:

| Uphill angle | Measured slip | BWR slip | Model minus measurement |
| --- | --- | --- | --- |
| 5 degrees | 20.0% | 37.6% | +17.6 percentage points |
| 10 degrees | 65.0% | 67.8% | +2.8 points |
| 12.5 degrees | 78.0% | 94.0% | +16.0 points |

MAE is 12.1 percentage points, versus 50.5 points for the earlier interpolation.
This improves agreement but fails the predeclared 10-point group MAE gate.
The gate also requires every error <=20 points and every observation solved.

Rigid grouser Husky, same mass/platform/campaign, 384 x 64 mm cylinder proxy:
the 10 and 12.5 degree solutions have about 10.3-point MAE after the conditional
nominal-to-tip slip-reference conversion. The 15-degree observation has **no
solution within the model's driving domain**, despite the measured rover
climbing. It remains failed and is not omitted to manufacture a passing score.
The two wheel configurations are not independent experimental campaigns.

The six-wheel 65.5 kg SSTB-Lite dune test is secondary: different natural sand,
one measured rear wheel, digitized points. Eight of ten points solve, conditional
MAE 11.5 points; two fail. Those statistics are not matched-soil validation of
this GRC-1 preset. No single aggregate score hides the different conditions.

Sensitivity shows `K` plus/minus its published SD and the NASA generic
compact-sand stress-shape row. Missing markers mean failed solutions. These
scenarios are not confidence bounds and do not replace the preset according to
which agrees best. The nominal-diameter rigid-cylinder sensitivity uses a common
384 mm slip reference when comparing results.

## Default adapter, slope requirements and report schema

`ModelSet().mobility` is `BekkerWongReeceMobility`; `ModelSet().slope_traversal`
is `SlopeTraversal`. Soil loading happens at setup, never in `compute()`.
Point-turn equations remain unchanged. The standalone fixed-design example can
still compare BWR straight driving with the legacy force law:

```powershell
.\.venv\Scripts\python.exe -m examples.bekker_wong_reece --check-derivatives
```

Configure `empty_slope_degrees` (default 30) and `loaded_slope_degrees` (default 20)
in `rover/config.py`, a case's assumptions, or `main.py --set NAME=VALUE`. Values
are degrees in `[0,90)`, fixed for each solve. Empty mass is dry mass; full mass is
dry plus the complete payload. These are additional checks, separate from the
existing 30-degree full-load static support requirement.

The acceptance rule is a solved driving slip strictly below 1. There is no 98%
cap, extra slip reserve, minimum ground speed or added motor constraint. For
both `empty` and `loaded`, report outputs `*_slope_slip`,
`*_slope_solution_valid`, `*_slope_traversal_margin`, `*_slope_force_shortfall`
(per wheel, N), `*_slope_sinkage` (m), and `*_slope_transfer_flag_count` preserve
the result. A failed driving solve reports slip=1 as a sentinel and validity=0,
not as a measured/predicted operating slip. Its margin is the negative force
deficit divided by normal wheel load (strictly negative even at the locked-wheel
boundary); a solved margin is `1-slip`. This signed failure closure gives the
optimizer useful derivatives in failed-demand trials. Final acceptance is strict,
without the normal feasibility tolerance. The HTML displays “No climbing solution”
for failures. Sinkage/flags for a failed demand describe the limiting contact,
not achieved traversal. Invalid contact geometry or soil still raises an error.

Identical route load/acceleration cases are batched explicitly within each model
evaluation; nothing is reused across evaluations or derivative perturbations.
For level straight segments, solve equal wheel loads first for zero net demand
(steady travel), then for `mass*acceleration/4`. The existing rolling-drag input
is not added, because the stress integration already contains compaction work.
With acceleration distance `da=peak_ground_speed^2/(2*acceleration)`:

```text
F_rim = 4*T_accel/r
F_rim0 = 4*T_steady/r
peak_rim_speed = peak_ground_speed/(1-s_accel)
work = F_rim*da/(1-s_accel)+F_rim0*(distance-da)/(1-s_steady)
peak_power = max(F_rim*peak_ground_speed/(1-s_accel),
                 F_rim0*peak_ground_speed/(1-s_steady))
```

This is a quasi-static acceleration/cruise closure with positive steady rolling
work retained during deceleration, no regeneration, and no wheel spin energy.
The net force solve already supplies chassis acceleration work; an extra
`0.5*mass*speed^2` must not be added. Gearmotor efficiency is applied once by the
existing machinery. Required motor torque is `T`, and motor speed uses rim
speed, not ground speed. The existing equal-torque friction screen is retained
as a separate conservative screen, not substituted for the BWR force balance.

Point turns, site relocation, and existing legacy pressure/stability screens
retain their reference equations and explicit dependencies. In particular,
`ground_pressure` still means the legacy deflection-footprint screening value;
it is not renamed as a BWR radial pressure. BWR pressure and sinkage have
distinct names in the diagnostic JSON. Only the two requested slip checks add constraints.

The normal report schema remains version 2. The example adds a
`bwr_slope_diagnostics` list with state, mass, slope, solver status, slip,
sinkage, force/torque, force residuals, pressure and transfer flags, or an
explicit failure reason. `bwr_diagnostic_scope` explains these standalone
checks. The report banner and evidence metadata label the default GRC-1 proxy and
its calibration status; the two new requirements are enforced in normal snapshots.

## Verification and remaining gaps

Tests cover independent adaptive integration in the zero-shear limit, normal
equilibrium, positive power dissipation, stress continuity and endpoints,
Mohr-Coulomb shear limits, load-dependent sinkage, recovery of an imposed-slip
force solution, smooth numerical sensitivities, invalid soil/geometry/gravity,
unsupported loads/demand, transfer flags, one-time work accounting and exact
point-turn invariance through ModelSet. Solvers use bracketed roots, not
convergence-dependent placeholders.

Before using this for design decisions, measure or independently source BP-1
pressure-sinkage and shear properties, stress-shape/rebound behavior, wheel
surface geometry, per-wheel loads, torque, rim/ground speed and preparation.
Conservatism is the student's chosen design assumption. The independent accuracy
gate remains failed despite default selection; software verification does not
establish prototype accuracy.
