"""Direct equations, no hidden discipline iteration. See MODEL.md for assumptions.

OpenMDAO passes one-element arrays for scalar variables. Calculations deliberately
use ordinary real scalars: finite differences support later non-complex-safe models.
"""

import math
import openmdao.api as om
from .config import input_spec
from .motor_inventory import ReferenceAssemblyMass


def scalar(inputs, name):
    return inputs[name].item()


class SimpleComponent(om.ExplicitComponent):
    """Only a small helper to avoid repeating input defaults and units."""

    def inputs_from_config(self, *names):
        for name in names:
            entry = input_spec(name)
            self.add_input(name, val=entry[0], units=entry[-1])

    def setup_partials(self):
        # Explicit structural dependencies, including BOTH sides of min/max branches.
        # These declarations make the N2 show only real possible input/output links.
        from .dependencies import dependencies
        mapping = getattr(self, "DEPENDENCIES", None)
        if mapping is None:
            for cls in type(self).__mro__:
                try:
                    mapping = dependencies(cls.__name__)
                    break
                except ValueError:
                    continue
        if mapping is None:
            raise ValueError(f"{type(self).__name__} needs a DEPENDENCIES map")
        for output, inputs in mapping.items():
            if inputs:
                self.declare_partials(output, sorted(inputs), method="fd", form="central", step=1e-5)


class GeometryMass(SimpleComponent):
    OUTPUT_UNITS = {
        "dry_mass": "kg", "drive_mass": "kg", "battery_mass": "kg",
        "rover_length": "m", "rover_width": "m", "rover_height": "m",
    }

    def initialize(self):
        self.options.declare("assembly_mass_model", default=ReferenceAssemblyMass)

    def setup(self):
        inventory = self.options["assembly_mass_model"]
        from .dependencies import dependencies
        self.DEPENDENCIES = {k:set(v) for k,v in dependencies("GeometryMass").items()}
        self.DEPENDENCIES["drive_mass"] = inventory.input_names("drive")
        self.DEPENDENCIES["dry_mass"] = inventory.input_names("drive") | {
            "battery_capacity", "battery_specific_energy", "fixed_mass", "structure_mass",
            "wheel_mass", "drum_mass", "linkage_mass"}
        self.inputs_from_config(
            "wheel_width", "drum_length", "battery_capacity", "track_width",
            "drum_pair_gap", "fixed_mass", "battery_specific_energy")
        self.inputs_from_config(*sorted(inventory.input_names("drive")))
        self.add_input("linkage_mass", units="kg")
        self.add_input("structure_mass", units="kg")
        self.add_input("wheel_mass", units="kg")
        self.add_input("drum_mass", units="kg")
        self.add_input("stowed_length", units="m")
        self.add_input("stowed_height", units="m")
        for name, units in self.OUTPUT_UNITS.items():
            self.add_output(name, units=units)

    def compute(self, inputs, outputs):
        v = lambda name: scalar(inputs, name)
        drive_mass = 4*self.options["assembly_mass_model"].per_motor(v, "drive")
        battery_mass = v("battery_capacity") / v("battery_specific_energy")
        outputs["drive_mass"], outputs["battery_mass"] = drive_mass, battery_mass
        outputs["dry_mass"] = v("fixed_mass") + v("structure_mass") + v("wheel_mass") + v("drum_mass") + drive_mass + battery_mass + v("linkage_mass")
        outputs["rover_length"] = v("stowed_length")
        outputs["rover_width"] = max(v("track_width") + v("wheel_width"),
                                     2 * v("drum_length") + v("drum_pair_gap"))
        outputs["rover_height"] = v("stowed_height")


def travel_profile(distance, speed, acceleration):
    """Rest-to-rest triangular/trapezoidal profile. Return time and peak speed."""
    peak = min(speed, math.sqrt(distance * acceleration))
    return distance / peak + peak / acceleration, peak


class Mission(SimpleComponent):
    def setup(self):
        self.inputs_from_config(
            "overhead_time", "electronics_power", "run_time",
            "berm_density", "delivery_efficiency", "battery_capacity",
            "battery_usable_fraction", "battery_c_rate", "mass_score_weight", "energy_score_weight", "available_excavation_area")
        for name, units in {
            "dry_mass": "kg", "payload_mass": "kg", "dig_time": "s", "dump_time": "s",
            "travel_time": "s", "turn_time": "s", "dig_power": "W", "dump_power": "W",
            "drive_peak_power": "W", "turn_peak_power": "W", "mobility_energy": "J",
            "actuation_time": "s", "arm_energy": "J", "arm_peak_power": "W",
            "dig_energy": "J", "dump_energy": "J", "dig_overlap_time": "s",
            "relocation_time": "s", "relocation_energy": "J", "relocation_peak_power": "W", "excavation_footprint": "m**2",
        }.items():
            self.add_input(name, val=0.0, units=units)
        for name, units in {
            "cycle_time": "s", "cycle_energy": "W*h", "cycles": None,
            "berm_volume": "m**3", "run_energy": "W*h", "average_power": "W",
            "peak_power": "W", "battery_energy_margin": "W*h", "battery_power_margin": "W",
            "mass_points": None, "energy_points": None, "construction_points": None,
            "complete_cycle_margin": "s",
            "excavation_area_margin": "m**2",
        }.items():
            self.add_output(name, units=units)

    def compute(self, inputs, outputs):
        v = lambda name: scalar(inputs, name)
        cycle = sum(v(n) for n in ("dig_time", "dump_time", "travel_time", "turn_time", "overhead_time", "actuation_time", "relocation_time"))-v("dig_overlap_time")
        energy_j = v("dig_energy") + v("dump_energy") + v("relocation_energy")
        energy_j += v("mobility_energy") + v("electronics_power") * cycle + v("arm_energy")
        cycles = v("run_time") / cycle  # continuous throughput approximation for optimization
        run_wh = cycles * energy_j / 3600
        volume = cycles * v("payload_mass") * v("delivery_efficiency") / v("berm_density")
        # Guidebook section 17.1 score table: time is in minutes, volume in cm^3.
        mass_points = 4.4 * volume * 1e6 / (v("run_time") / 60 * v("dry_mass"))
        energy_points = 1.5 * volume * 1e6 / (v("run_time") / 60 * run_wh)
        peak = v("electronics_power") + max(v("dig_power")+v("arm_peak_power"), v("dump_power"), v("drive_peak_power"),
                                            v("turn_peak_power"), v("relocation_peak_power"))
        outputs["cycle_time"], outputs["cycle_energy"], outputs["cycles"] = cycle, energy_j / 3600, cycles
        outputs["berm_volume"], outputs["run_energy"] = volume, run_wh
        outputs["average_power"], outputs["peak_power"] = run_wh * 3600 / v("run_time"), peak
        outputs["battery_energy_margin"] = v("battery_capacity") * v("battery_usable_fraction") - run_wh
        outputs["battery_power_margin"] = v("battery_capacity") * v("battery_c_rate") - peak
        outputs["mass_points"], outputs["energy_points"] = mass_points, energy_points
        outputs["construction_points"] = v("mass_score_weight") * mass_points + v("energy_score_weight") * energy_points
        outputs["complete_cycle_margin"] = v("run_time") - cycle
        outputs["excavation_area_margin"] = v("available_excavation_area")-cycles*v("excavation_footprint")
