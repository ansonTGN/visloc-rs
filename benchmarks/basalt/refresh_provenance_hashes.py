"""Refresh checked-in generator hashes after an authorized source edit.

This intentionally updates only the ``sha256`` values in
``basalt_provenance_manifest_v1.json`` and preserves the manifest's layout.
It covers checked-in generators, explicitly retained audit sources, and any
production port-file records that opt into a source hash.  Run it once sibling
agents have finished editing the audited sources, then run
``verify_provenance.py``.
"""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "benchmarks/basalt/basalt_provenance_manifest_v1.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def refresh_records(
    text: str,
    records: dict[str, object],
    *,
    only_hashed: bool = False,
) -> tuple[str, list[str]]:
    changed: list[str] = []
    for relative, record in records.items():
        if not isinstance(record, dict):
            raise SystemExit(f"record is not an object: {relative}")
        if only_hashed and "sha256" not in record:
            continue
        path = ROOT / relative
        if not path.is_file():
            raise SystemExit(f"missing audited file: {relative}")
        current = sha256(path)
        old = record["sha256"]
        if current == old:
            continue
        pattern = re.compile(
            rf'("{re.escape(relative)}"\s*:\s*\{{\s*"(?:sha256|translation_status)"\s*:\s*")'
        )
        if "sha256" not in record:
            raise SystemExit(f"record has no hash field: {relative}")
        pattern = re.compile(
            rf'("{re.escape(relative)}"\s*:\s*\{{[^}}]*?"sha256"\s*:\s*")[0-9a-f]{{64}}(")',
            re.DOTALL,
        )
        updated, count = pattern.subn(rf"\g<1>{current}\g<2>", text, count=1)
        if count != 1:
            raise SystemExit(f"could not locate hash entry for {relative}")
        text = updated
        changed.append(f"{relative}: {old} -> {current}")
    return text, changed


def main() -> int:
    text = MANIFEST.read_text(encoding="utf-8")
    payload = json.loads(text)
    records = payload["checked_in_generator_hashes"]
    text, changed = refresh_records(text, records)
    retained = payload.get("retained_audit_sources", {})
    if not isinstance(retained, dict):
        raise SystemExit("retained_audit_sources must be an object")
    text, retained_changed = refresh_records(text, retained)
    changed.extend(retained_changed)
    port_files = payload.get("port_files", {})
    if not isinstance(port_files, dict):
        raise SystemExit("port_files must be an object")
    text, source_changed = refresh_records(text, port_files, only_hashed=True)
    changed.extend(source_changed)
    if changed:
        MANIFEST.write_text(text, encoding="utf-8", newline="\n")
    print("updated" if changed else "already current")
    for item in changed:
        print(item)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
