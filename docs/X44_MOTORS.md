# Standard X44 motor profile — physics revision 2, simplified interface revision 3

Select `--motors x44-standard` alongside `--structure plate-frame`, or
`build_problem(structure="plate-frame", motors="x44-standard")`.
The generic `ModelSet()` and motor configuration remain available as reference.
The wheelbase upper bound is now 0.65 m by student request, with the original
1.50 m overall stowed-length requirement unchanged.

## Hardware and evidence

Four drive motors and two drum motors are fixed standard Kraken X44s at 12 V.
Drive reduction is fixed at 100:1 (4×5×5 MAXPlanetary). Drum reduction remains
an exploratory continuous variable between 9 and 125, initially 75. Select a
real cartridge combination and rerun before purchasing/using a reported ratio;
the optimization does not establish that a noninteger ratio exists. Cartridge
configuration ratings and the motor input adapter still require verification.

The small official [CTR curve CSV](https://motors.ctr-electronics.com/assets/motor_data/data_Kraken%20X44.csv)
is versioned in `data/motors/`, with source, date, units, checksum, domain and
validation status in `manifest.json`. Its last-updated index is 2025-09-26.
The [WCP performance table](https://docs.wcproducts.com/welcome/electronics/kraken-x44/kraken-x44-motor/overview-and-features/motor-performance)
distinguishes standard from FOC and explains that peak power is not a continuous
robot operating rating. The standard CSV endpoints are 7757.60474 rpm free speed,
4.113357 N m stall torque, 279.09869 A stall current and 3.15605 A free current.

The [WCP physical specification](https://docs.wcproducts.com/welcome/electronics/kraken-x44/kraken-x44-motor/overview-and-features/physical-specifications.md)
lists 0.75 lb = 0.34019428 kg including Talon FX. The
[REV inventory](https://www.revrobotics.com/maxplanetary-system-kit/) gives drive
gearbox mass 0.697 kg including the listed output shaft. The undecided drum
gearbox uses a clearly provisional 0.753 kg three-stage inventory budget.
It does not shrink when the optimizer lowers ratio. Weigh the actual assemblies
and update these assumptions when adapters, stages and shaft lengths are fixed.
Static failure torques on the REV page are not used as continuous allowables.

## Equations and domain

Let T_s, I_s, I_0 and w_0 be the published endpoints. Extract coefficients from
the first/last rows only; the interior rows are reserved for reproduction checks.

```text
T_available(rpm) = T_s (1 - rpm/rpm_free)
Kt = T_s / (I_s - I_0)
T_current_cap = Kt (I_limit - I_0)  # only when a limit is specified
E_electrical = W_output / eta_combined
P_electrical_peak = P_output_peak / eta_combined
```

Torque-speed capability comes from the official 12 V curve. Current endpoints
only support an optional torque cap; no current time history is calculated.
Mission electrical energy uses explicitly provisional combined efficiencies:
drive 0.72 and drum 0.70. These are editable assumptions, not fits to this CSV,
not the manufacturer's maximum efficiency, and not experimentally validated.
They are coarse planning estimates, especially weak near stall/no-load. No
continuous holding-current, PWM, copper-loss, voltage or thermal model remains.

Gear conversion is w_motor=G w_output and T_motor=T_output/(G eta_gear).
Gear efficiency remains the provisional 0.80 input, not MAXPlanetary test data.
When a stator limit is specified, available torque is also capped by
Kt(I_limit-I_0). A zero current-limit input explicitly means **undecided**;
it enables only the 12 V instantaneous envelope and never claims a continuous
rating. The profile supports only 12 V and the selected 100:1 drive ratio.
There is no FOC substitution.

Outside the torque-speed/current envelope, signed torque or force margins
fail. Negative available torque above free speed denotes a failed screen, not
an extrapolated operating prediction. A finite work estimate does not make
that point feasible. Nonfinite/negative work and invalid gearbox or combined
efficiencies are rejected. Peak power is a necessary instantaneous curve screen,
not a continuous rating; torque-speed/current margins remain enforced too.

## Mission integration and interfaces

The same Excavation, Mobility, SiteRelocation, DrumMass and GeometryMass classes
are used for both motor profiles. Motor-provider and assembly-inventory options
preserve required scalar output names, native units, mass boundaries and
positive-margin conventions. No X44-specific discipline copies remain.
See [MOTOR_INTERFACE.md](MOTOR_INTERFACE.md) for the file map and Python API.
The hardware profile removes both motor-power design variables and freezes
drive ratio. Legacy generic power/rated-rpm/mass coefficients remain
in config/reports as inactive legacy fields; they are not consumer input ports
and have no effect on hardware-profile outputs. Combined efficiencies are active.

Drum mass is four reference shells plus **two** actual motor/gear inventories.
Drive mass is **four** actual motor/gear inventories; GeometryMass adds them
once. Drum shell physics remains uncalibrated. No fitting changes wheel mass.

Drive work sums resistance times distance and startup kinetic energy for
each existing segment of the fixed 45 s route, deposition turn and relocation.
Turning uses equivalent mass Izz/(track/2)^2. No regenerative braking is credited.
Wheel/rotor inertia and braking hardware are not verified. Electrical energy
is mechanical work / combined efficiency; peaks use the same conversion.

Excavation keeps the existing finite soil, storage, force, timing and service
factor. Dig work uses half-full mean internal torque; dump work uses a mean
unloading torque. Startup retains conservative loaded thin-ring inertia:
1/2 I omega^2 work and I omega / spinup_time torque. Work is counted for both
pairs, which dig together and dump separately. The 1.25 service factor applies
to torque screens; energy uses modeled loads without that factor.

Combined efficiency already includes gear losses; the mechanical gearbox
efficiency is used separately for capability and is not applied to energy again.
Electronics/actuator idle and arm energy stay in their existing locations, once.

Optional peak/RMS current and required-voltage report fields have been removed.
The report records energy model revision 2 and combined efficiencies and marks
continuous operation unverified. Slope/rock stability is still static;
**this update does not verify climbing** or improve the optimizer.

## Verification and remaining evidence

Tests compare interior CSV points to the endpoint-defined equations, check
mechanical envelope limits, independent constant-force work, zero/stall/free
limits, overspeed/current-limit failures, gearbox conversion, real mass sums,
reference defaults, explicit N2 sources and finite-difference derivatives.
The separate rounded WCP 41 A specification is a cross-check, not an independent
experiment. Manufacturer curve reproduction and optimizer success do not
validate constant-efficiency energy estimates, thermal operation, gearbox
efficiency, soil or actual rover loads.
Current limits and a loaded test duty cycle are the next required hardware data.
