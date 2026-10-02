"""Reject incomplete replacements before OpenMDAO supplies hidden input defaults.

The starter promotes scalar ports by their contract names. Required outputs must
retain the native units used in reports. Additional inputs need an explicit source
in config or another discipline; simulator adapters may convert internally.
"""
import openmdao.api as om
from openmdao.utils.units import unit_conversion
from .models import ModelSet


def same_native_units(actual, expected):
    if actual is None or expected is None:
        return actual is expected
    try:
        factor, offset = unit_conversion(actual, expected)
        return abs(factor-1)<1e-12 and abs(offset)<1e-12
    except (TypeError, ValueError):
        return False


def validate_interfaces(problem):
    """Call after setup, before run_model/final_setup creates any auto-IVC."""
    sources = {}
    components = list(problem.model.system_iter(recurse=True, typ=om.ExplicitComponent))
    for component in components:
        for name, meta in component.get_io_metadata(
                iotypes="output", metadata_keys=["units", "shape"], return_rel_names=True).items():
            promoted = meta["prom_name"]
            if promoted in sources:
                raise ValueError(f"Duplicate source for {promoted}; preserve unique promoted scalar ports")
            if meta["shape"] != (1,):
                raise ValueError(f"{component.pathname}.{name}: contract ports must be scalar")
            sources[promoted] = meta

    for slot, reference in vars(ModelSet()).items():
        component = getattr(problem.model, slot)
        outputs = component.get_io_metadata(
            iotypes="output", metadata_keys=["units", "shape"], return_rel_names=True)
        for name, units in reference.OUTPUT_UNITS.items():
            if name not in outputs:
                raise ValueError(f"Replacement {slot} is missing required output {name}")
            if not same_native_units(outputs[name]["units"], units):
                raise ValueError(f"Replacement {slot}.{name} must use native units {units!r}; convert inside its adapter")

    for component in components:
        for name, meta in component.get_io_metadata(
                iotypes="input", metadata_keys=["units", "shape"], return_rel_names=True).items():
            promoted = meta["prom_name"]
            if promoted not in sources:
                raise ValueError(f"No explicit source for {component.pathname}.{name}; add a documented config or coupled input")
            if meta["shape"] != (1,) or not same_native_units(meta["units"], sources[promoted]["units"]):
                raise ValueError(f"{component.pathname}.{name}: scalar/native-unit contract mismatch")
