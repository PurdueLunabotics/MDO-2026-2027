"""Four drum shells plus the selected two motor/gear assembly inventories."""
import math
from .components import scalar
from .discipline import Discipline, names
from .motor_inventory import ReferenceAssemblyMass, reference_assembly_mass


def reference_drum_mass(diameter, length, areal_mass, fixed_mass_per_pair,
                        motor_mass_per_watt, motor_power_per_pair,
                        gear_mass_per_torque, motor_rated_rpm, gear_ratio,
                        gear_efficiency):
    """Four shells plus two motor/gear assemblies [kg]; uncalibrated."""
    shell = 4 * areal_mass * (math.pi * diameter * length + math.pi * diameter**2 / 2)
    return shell + 2*reference_assembly_mass(fixed_mass_per_pair, motor_mass_per_watt,
        motor_power_per_pair, gear_mass_per_torque, motor_rated_rpm, gear_ratio, gear_efficiency)


class DrumMass(Discipline):
    CONFIG_INPUTS = tuple(sorted(names(
        "drum_diameter drum_length drum_areal_mass drum_fixed_mass "
        "motor_mass_per_watt drum_motor_power gear_mass_per_torque "
        "motor_rated_rpm drum_gear_ratio drum_gear_efficiency")))
    OUTPUT_UNITS = {"drum_mass": "kg"}
    DEPENDENCIES = {"drum_mass": set(CONFIG_INPUTS)}

    def initialize(self):
        self.options.declare("assembly_mass_model", default=ReferenceAssemblyMass)

    def setup(self):
        inventory = self.options["assembly_mass_model"]
        self.CONFIG_INPUTS = tuple(sorted({"drum_diameter", "drum_length", "drum_areal_mass"}
                                          | inventory.input_names("drum")))
        self.DEPENDENCIES = {"drum_mass": set(self.CONFIG_INPUTS)}
        super().setup()

    def compute(self, inputs, outputs):
        v = lambda n: scalar(inputs, n)
        shell = 4*v("drum_areal_mass")*(math.pi*v("drum_diameter")*v("drum_length")
                                      + math.pi*v("drum_diameter")**2/2)
        outputs["drum_mass"] = shell+2*self.options["assembly_mass_model"].per_motor(v,"drum")
