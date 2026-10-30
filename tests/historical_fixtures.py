"""Original actuator search space for historical numerical/geometry regressions.

Use only inside explicit test patches. Current hardware ranges in production
config remain enforced and have separate boundary/freeze coverage.
"""

HISTORICAL_ACTUATOR_DESIGN = {
    "travel_actuator_length": (0.38, 0.30, 0.65, "m"),
    "actuator_retracted_length": (0.45, 0.25, 0.65, "m"),
    "actuator_extended_length": (0.68, 0.45, 1.05, "m"),
}
