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
        extra = set(data)-{"name", "revision", "structure", "motors", "actuators", "traversal_mode", "attachment_max", "inputs", "notes"}
        if extra:
            raise ValueError(f"Unknown case fields: {', '.join(sorted(extra))}")
        values = data["inputs"]
        metadata = {n: data[n] for n in ("name", "revision", "structure", "motors", "actuators", "traversal_mode") if n in data}
        if any(not isinstance(value, str) for value in metadata.values()):
            raise ValueError("Case name, revision, hardware profiles and traversal mode must be strings.")
        if 'attachment_max' in data:
            value=data['attachment_max']
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 1<=value<=1.5:
                raise ValueError('Case attachment_max must be between 1 and 1.5')
            metadata['attachment_max']=value
        if "notes" in data:
            metadata["notes"] = data["notes"]
    else:
        values, metadata = data, {}
    if not isinstance(values, dict):
        raise ValueError("Case inputs must be a name-to-number object.")
    for name, value in values.items():
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
            raise ValueError(f"Case input {name!r} must be a finite number.")
    values = dict(values)
    if "actuator_attachment_fraction" in values:
        if {"lin_act_side_arm_offset", "lin_act_top_arm_offset"} & values.keys():
            raise ValueError("Case mixes old fraction and new actuator offsets")
        if "arm_length" not in values:
            raise ValueError("Legacy fraction migration needs explicit arm_length")
        f = values.pop("actuator_attachment_fraction")
        if f <= 0: raise ValueError("Legacy attachment fraction must be positive")
        values.update(lin_act_side_arm_offset=(f-1)*values["arm_length"], lin_act_top_arm_offset=0.)
        metadata["attachment_migration"] = "Legacy fraction converted to side=(f-1)*arm_length, top=0 m"
        metadata["notes"] = {**(metadata.get("notes") or {}), "attachment_migration": metadata["attachment_migration"]}
    return values, {**metadata, "source": path.name, "sha256": hashlib.sha256(raw).hexdigest()}
