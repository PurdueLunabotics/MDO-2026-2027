"""Run: python -m examples.empirical_excavation (from the project root).

Demonstrates component injection and strict fixed-condition interpolation. Do not
run a full geometry optimization with this one-dimensional demonstration table.
"""
from pathlib import Path
import hashlib
import openmdao.api as om
from rover.calibration import ResponseTable
from rover.components import scalar
from rover.config import DESIGN, ASSUMPTIONS
from rover.excavation import Excavation
from rover.models import ModelSet
from rover.problem import build_problem


class TableExcavation(Excavation):
    def initialize(self):
        super().initialize()
        self.options.declare("table_path", types=(str,Path))
        self.options.declare("allow_synthetic", default=False, types=bool)

    def setup(self):
        self.table=ResponseTable(self.options["table_path"],allow_synthetic=self.options["allow_synthetic"])
        self.model_evidence = {
            "data_kind": self.table.data["data_kind"], "revision": self.table.data["revision"],
            "source": self.table.data["source"], "validation": self.table.data["validation"],
            "data_sha256": hashlib.sha256(self.table.path.read_bytes()).hexdigest(),
            "conditions": self.table.data["conditions"],
            "froude_domain": self.table.rows[[0,-1],0].tolist(),
        }
        # A dimensionless column does not make the response geometry-independent.
        for name,spec in self.table.data["conditions"].items():
            config=DESIGN.get(name,ASSUMPTIONS.get(name))
            if name not in self.CONFIG_INPUTS or config is None or spec["units"] != (config[-1] or "1"):
                raise ValueError(f"Unknown condition or incompatible units: {name}")
        super().setup()

    def granular_response(self, inputs, froude):
        try:
            return self.table.evaluate(froude,{n:scalar(inputs,n) for n in self.table.data["conditions"]})
        except ValueError as exc:
            raise om.AnalysisError(str(exc)) from exc


def demonstration_model():
    return TableExcavation(table_path=Path(__file__).parent/"data"/"synthetic_response.json",allow_synthetic=True)


def main():
    p=build_problem(models=ModelSet(excavation=demonstration_model))
    try:
        p.run_model()
        print("SYNTHETIC ADAPTER DEMONSTRATION — not measured BP-1 performance")
        print(f"Payload: {p.get_val('payload_mass').item():.3f} kg")
    finally:
        p.cleanup()


if __name__ == "__main__":
    main()
