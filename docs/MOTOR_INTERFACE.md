# The simple motor interface

Each discipline calculates its own loads. One shared helper applies gearing,
checks the selected motor envelope, and converts mechanical work to electrical
energy. There are no electrical phase histories, PWM simulation or RMS currents.

| File | What to change here |
| --- | --- |
| `rover/alternatives/x44_motor.py` | Standard X44 torque-speed curve and optional current cap |
| `rover/motors.py` | Shared gear conversion and work / efficiency accounting |
| `rover/excavation.py` | Soil loads, drum torque, startup work and excavation time |
| `rover/mobility.py` | Route, resistance, acceleration, turning and traction |
| `rover/relocation.py` | Movement to fresh ground |
| `rover/motor_inventory.py` | Motor and gearbox assembly mass |
| `rover/config.py` | Ratios, provisional efficiencies, current limits and catalog masses |

## A small example

```python
from rover.alternatives.x44_motor import CURVE
from rover.motors import Gearmotor

drive = Gearmotor(CURVE, ratio=100, efficiency=0.80,
                  electrical_efficiency=0.72)
available = drive.available_torque(rpm=50)  # output-shaft N*m
result = drive.evaluate(work=72, peak=36)   # output-shaft J and W, ONE motor
print(result)  # {"energy": 100.0, "peak": 50.0}, electrical J and W
```

The two efficiencies have different jobs:

- `efficiency` is mechanical gearbox efficiency, used for available torque.
- `electrical_efficiency` includes motor AND gearbox losses, used for energy
  and peak electrical power. It is applied once.

In config, `drive_efficiency=0.72` and `drum_efficiency=0.70` are provisional
combined efficiencies, active in both motor profiles. They are not measured X44
efficiencies. No current, temperature or continuous rating is predicted.

The discipline accounts for motor count and adds auxiliary loads separately:
four wheel motors, two digging motors, one dumping pair/motor at a time.
Work includes resistance work and startup kinetic energy, with no regeneration.

## Selecting or replacing a motor

The CLI remains `--motors x44-standard`. Existing Excavation, Mobility and
SiteRelocation components receive a `motor_model` option. DrumMass and
GeometryMass receive an `assembly_mass_model` option. ModelSet still selects
the discipline classes, including a student's custom excavation model.

A provider implements `input_names(role)` and
`create(get, role, rated_torque_limit=...)`. It returns a Gearmotor.
The motor curve only needs `free_rpm`, `available_torque(rpm)`,
`peak_power()` and `speed_limit(torque)`. Pure helpers do no file or network IO.

For a new curve, change its module/provider; do not copy excavation or mobility.
For changed soil or route loads, change that discipline; do not edit the curve.
Subclasses overriding `initialize()` must call `super().initialize()`.

## What changed in this simplification

The old internal ShaftDuty/ShaftPhase and IntegratedMotor APIs and
`drive_motion.py` are removed. `Gearmotor.evaluate(work, peak)` now takes two
scalars. Required OpenMDAO ports, units, mass boundaries and margin signs stay
the same. Optional current/voltage report fields are retired explicitly;
metadata records energy model revision 2 and the selected efficiencies.

The original generic results are preserved. X44 mechanical demands, mass and
torque-speed screens are preserved at the same inputs; energy, electrical
peak power, battery margins and score intentionally change. Historical results
are not rewritten. The old fixture remains regression evidence for reference
results and X44 mechanical invariants, not an expected X44 electrical answer.

Tests cover manufacturer-curve reproduction, independent work/kinetic-energy
limits, invalid inputs, current-limit failure, gearbox conversion, one-time
loss accounting, shared-provider replacement, mass/mission sums and derivatives.
See [X44_MOTORS.md](X44_MOTORS.md) for hardware evidence and limitations.
