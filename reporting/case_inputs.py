"""Read a named design case before running the model. Never used by compute()."""
import hashlib
import json
import math
from pathlib import Path


def read_case(path):
    path = Path(path)
    raw = path.read_bytes()
    data = json.loads(raw.decode("utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError("Input JSON must be an object containing numeric inputs.")
    if "inputs" in data:
        extra = set(data)-{"name", "revision", "structure", "motors", "inputs", "notes"}
        if extra:
            raise ValueError(f"Unknown case fields: {', '.join(sorted(extra))}")
        values = data["inputs"]
        metadata = {n: data[n] for n in ("name", "revision", "structure", "motors") if n in data}
        if any(not isinstance(value, str) for value in metadata.values()):
            raise ValueError("Case name, revision, structure and motors must be strings.")
        if "notes" in data:
            metadata["notes"] = data["notes"]
    else:
        values, metadata = data, {}
    if not isinstance(values, dict):
        raise ValueError("Case inputs must be a name-to-number object.")
    for name, value in values.items():
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
            raise ValueError(f"Case input {name!r} must be a finite number.")
    return values, {**metadata, "source": path.name, "sha256": hashlib.sha256(raw).hexdigest()}
