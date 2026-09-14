"""Small dependency-free JSON Schema validator for Basalt evidence files.

The benchmark suite intentionally does not add a Python package dependency just
to validate its checked-in evidence.  The schemas used here only need the
draft-2020 constructs present in the two Basalt schemas, so this module
implements that constrained subset and reports the JSON path of failures.
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any


SCHEMA_ROOT = Path(__file__).resolve().parent / "schemas"


def _type_matches(value: Any, expected: str) -> bool:
    if expected == "object":
        return isinstance(value, dict)
    if expected == "array":
        return isinstance(value, list)
    if expected == "string":
        return isinstance(value, str)
    if expected == "boolean":
        return isinstance(value, bool)
    if expected == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if expected == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if expected == "null":
        return value is None
    return False


def _describe(path: str, message: str) -> str:
    return f"{path}: {message}"


def validate_json_schema(value: Any, schema: dict[str, Any], *, root: dict[str, Any] | None = None, path: str = "$",) -> list[str]:
    """Return validation errors for the constrained checked-in schema subset."""

    root = schema if root is None else root
    errors: list[str] = []
    reference = schema.get("$ref")
    if reference:
        if not reference.startswith("#/"):
            return [_describe(path, f"unsupported reference {reference!r}")]
        target: Any = root
        for part in reference[2:].split("/"):
            target = target.get(part) if isinstance(target, dict) else None
        if not isinstance(target, dict):
            return [_describe(path, f"unresolved reference {reference!r}")]
        return validate_json_schema(value, target, root=root, path=path)

    if "const" in schema and value != schema["const"]:
        errors.append(_describe(path, f"must equal {schema['const']!r}"))
    if "enum" in schema and value not in schema["enum"]:
        errors.append(_describe(path, f"must be one of {schema['enum']!r}"))

    if "anyOf" in schema and not any(
        not validate_json_schema(value, option, root=root, path=path)
        for option in schema["anyOf"]
    ):
        errors.append(_describe(path, "does not match anyOf"))
    if "not" in schema and not validate_json_schema(value, schema["not"], root=root, path=path):
        errors.append(_describe(path, "matches forbidden schema"))

    expected_type = schema.get("type")
    if expected_type is not None:
        types = expected_type if isinstance(expected_type, list) else [expected_type]
        if not any(_type_matches(value, item) for item in types):
            errors.append(_describe(path, f"has type {type(value).__name__}, expected {types!r}"))
            return errors

    if isinstance(value, str):
        if "minLength" in schema and len(value) < schema["minLength"]:
            errors.append(_describe(path, "is shorter than minLength"))
        if "pattern" in schema and re.fullmatch(schema["pattern"], value) is None:
            errors.append(_describe(path, "does not match pattern"))

    if isinstance(value, (int, float)) and not isinstance(value, bool):
        try:
            finite = math.isfinite(float(value))
        except OverflowError:
            finite = False
        if not finite:
            errors.append(_describe(path, "must be finite"))
        if "minimum" in schema and value < schema["minimum"]:
            errors.append(_describe(path, "is below minimum"))
        if "exclusiveMinimum" in schema and value <= schema["exclusiveMinimum"]:
            errors.append(_describe(path, "does not exceed exclusiveMinimum"))

    if isinstance(value, list):
        if "minItems" in schema and len(value) < schema["minItems"]:
            errors.append(_describe(path, "has fewer than minItems"))
        item_schema = schema.get("items")
        if isinstance(item_schema, dict):
            for index, item in enumerate(value):
                errors.extend(validate_json_schema(item, item_schema, root=root, path=f"{path}[{index}]"))

    if isinstance(value, dict):
        if "minProperties" in schema and len(value) < schema["minProperties"]:
            errors.append(_describe(path, "has fewer than minProperties"))
        for required in schema.get("required", []):
            if required not in value:
                errors.append(_describe(path, f"missing required property {required!r}"))
        properties = schema.get("properties", {})
        if isinstance(properties, dict):
            for key, child_schema in properties.items():
                if key in value and isinstance(child_schema, dict):
                    errors.extend(validate_json_schema(value[key], child_schema, root=root, path=f"{path}.{key}"))
        # The provenance schema uses object-level ``additionalProperties`` to
        # apply a $defs validator to every dynamically named upstream/file.
        # Enforce that constrained subset instead of silently accepting those
        # values as opaque extras.
        additional = schema.get("additionalProperties")
        if isinstance(additional, dict):
            known = set(properties) if isinstance(properties, dict) else set()
            for key, child in value.items():
                if key not in known:
                    errors.extend(validate_json_schema(child, additional, root=root, path=f"{path}.{key}"))
        elif additional is False:
            known = set(properties) if isinstance(properties, dict) else set()
            for key in value:
                if key not in known:
                    errors.append(_describe(f"{path}.{key}", "is not allowed"))
    return errors


def schema_path(name: str) -> Path:
    path = SCHEMA_ROOT / name
    if not path.is_file():
        raise ValueError(f"unknown Basalt schema: {name}")
    return path


def validate_document(value: Any, schema_name: str) -> None:
    path = schema_path(schema_name)
    schema = json.loads(path.read_text(encoding="utf-8"))
    errors = validate_json_schema(value, schema)
    if errors:
        raise ValueError("schema validation failed: " + "; ".join(errors[:8]))


def validate_document_file(path: Path, schema_name: str) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    validate_document(value, schema_name)
    return value
