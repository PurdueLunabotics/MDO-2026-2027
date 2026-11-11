"""Opt-in development controls; no physics or persistent compute caches."""
import cProfile
import io
import math
import pstats
import time

from openmdao.drivers.scipy_optimizer import ScipyOptimizeDriver


class RunBudgetExceeded(RuntimeError):
    """A bounded development run stopped before optimizer convergence."""


class BudgetedScipyDriver(ScipyOptimizeDriver):
    """COBYLA budgets checked between complete objective evaluations.

    A single model evaluation cannot be interrupted by the time limit. Checking
    before the next callback keeps the final inputs and outputs consistent.
    """
    def __init__(self, *, evaluation_limit, time_limit, **options):
        if (isinstance(evaluation_limit, bool) or not isinstance(evaluation_limit, int)
                or evaluation_limit < 1):
            raise ValueError("Evaluation limit must be a positive integer")
        if not math.isfinite(time_limit) or time_limit <= 0:
            raise ValueError("Time limit must be positive and finite")
        if options.get("optimizer", "SLSQP") != "COBYLA":
            raise ValueError("Development time/evaluation budgets require --optimizer COBYLA")
        super().__init__(**options)
        self.evaluation_limit = evaluation_limit
        self.time_limit = time_limit
        self.objective_evaluations = 0
        self._deadline = None

    def run(self):
        self.objective_evaluations = 0
        self._deadline = time.perf_counter() + self.time_limit
        return super().run()

    def _objfunc(self, x_new):
        if self.objective_evaluations >= self.evaluation_limit:
            self.fail = True
            raise RunBudgetExceeded(f"Development evaluation limit reached ({self.evaluation_limit})")
        if self._deadline is not None and time.perf_counter() >= self._deadline:
            self.fail = True
            raise RunBudgetExceeded(f"Development time limit reached ({self.time_limit:g} s)")
        result = super()._objfunc(x_new)
        self.objective_evaluations += 1
        return result


def selected_freeze(design, *, vary=None, freeze=()):
    """Keep only named active variables free, preserving original bound checks."""
    active, frozen = set(design), set(freeze)
    if vary is not None:
        selected = set(vary)
        unknown = selected-active
        if unknown:
            raise ValueError("--vary requires active design variables; unknown or fixed: "
                             + ", ".join(sorted(unknown)))
        if not selected:
            raise ValueError("--vary needs at least one design variable")
        if selected & frozen:
            raise ValueError("A variable cannot appear in both --vary and --freeze")
        frozen |= active-selected
    return sorted(frozen)


def profile_model(problem):
    """Profile exactly one complete evaluation; return time and cumulative stats."""
    profiler = cProfile.Profile()
    start = time.perf_counter()
    profiler.runcall(problem.run_model)
    elapsed = time.perf_counter()-start
    output = io.StringIO()
    pstats.Stats(profiler, stream=output).sort_stats("cumulative").print_stats(25)
    return elapsed, output.getvalue()
