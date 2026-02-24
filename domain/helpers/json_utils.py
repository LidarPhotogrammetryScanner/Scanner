import json
from dataclasses import is_dataclass, fields, asdict
import json
from dataclasses import dataclass, field, fields, is_dataclass
from typing import Any, Dict, List, Type, TypeVar, get_origin, get_args


# Converts a python object into json
def to_json(obj) -> str:
    return json.dumps(asdict(obj), separators=(",", ":"))

# Try to convert a json string into a python object


T = TypeVar('T')

def _convert_value(value: Any, target_type: Type) -> Any:
    """Try to convert primitive value to target_type. Return None if fails."""
    try:
        if target_type in (int, float, str, bool):
            return target_type(value)
        return value
    except (ValueError, TypeError):
        return None

def from_json(cls: Type[T], data: Any) -> T:
    """
    Recursively convert a dict/list to a dataclass instance or primitive.
    Handles nested dataclasses, lists, dicts, and primitives at any depth.
    Converts primitives to their declared type, skips on conversion failure.
    """
    if is_dataclass(cls):
        if not isinstance(data, dict):
            return None  # Cannot convert non-dict to dataclass
        init_values = {}
        for f in fields(cls):
            if f.name not in data:
                continue
            value = from_json(f.type, data[f.name])
            if value is not None:
                init_values[f.name] = value
        return cls(**init_values)

    origin = get_origin(cls)
    args = get_args(cls)

    # Handle lists
    if origin in (list, List):
        elem_type = args[0] if args else Any
        if isinstance(data, list):
            return [from_json(elem_type, v) for v in data if from_json(elem_type, v) is not None]
        return []

    # Handle dicts
    if origin in (dict, Dict):
        key_type, val_type = args if args else (Any, Any)
        if isinstance(data, dict):
            result = {}
            for k, v in data.items():
                conv_key = from_json(key_type, k)
                conv_val = from_json(val_type, v)
                if conv_key is not None and conv_val is not None:
                    result[conv_key] = conv_val
            return result
        return {}

    # Handle primitives
    return _convert_value(data, cls)