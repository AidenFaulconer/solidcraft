#!/usr/bin/env python3
"""A/B test: SolidCraft STL→STEP (A) vs name-matched reference STEP (B).

For each ``*.stl`` in the fixtures directory that has a sibling ``*.step`` with the
same basename, convert with ``file.insert_mesh_solid``, export STEP, and compare
against the reference:

* conversion success / error text
* volume and surface area from ``solidcraft-cli eval`` (when both import)
* STEP entity-type histogram (planes, cylinders, …)
* optional OCCT xor via skills/solidcraft-checking/scripts/compare_step.py when
  ``uv`` + build123d are available

Exit 0 only when every case converts and meets ``--volume-min`` similarity
(default 0.95). Convert failures are reported as FAIL (expected while the
prismatic converter is incomplete).

Examples::

  python run_ab_tests.py --cli path/to/solidcraft-cli.exe
  python run_ab_tests.py --case PSU_rear_mount_back_side_1-screw_v2
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any

ENTITY_RE = re.compile(r"\b([A-Z][A-Z0-9_]{2,})\b")
# Public STEP geometry / topology tokens worth comparing (not boilerplate).
ENTITY_KEEP = {
    "ADVANCED_FACE",
    "B_SPLINE_CURVE_WITH_KNOTS",
    "B_SPLINE_SURFACE_WITH_KNOTS",
    "CARTESIAN_POINT",
    "CIRCLE",
    "CLOSED_SHELL",
    "CONICAL_SURFACE",
    "CYLINDRICAL_SURFACE",
    "EDGE_CURVE",
    "EDGE_LOOP",
    "FACE_BOUND",
    "FACE_OUTER_BOUND",
    "LINE",
    "MANIFOLD_SOLID_BREP",
    "PLANE",
    "SPHERICAL_SURFACE",
    "TOROIDAL_SURFACE",
    "VERTEX_POINT",
}


def discover_cases(fixtures: Path) -> list[dict[str, Path]]:
    cases: list[dict[str, Path]] = []
    for stl in sorted(fixtures.glob("*.stl")):
        step = stl.with_suffix(".step")
        if not step.is_file():
            # also accept .stp
            alt = stl.with_suffix(".stp")
            step = alt if alt.is_file() else step
        if step.is_file():
            cases.append({"name": stl.stem, "stl": stl, "reference": step})
    return cases


def run(cmd: list[str], *, cwd: Path | None = None, timeout: float = 600) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=timeout,
        check=False,
    )


def write_convert_script(path: Path, stl: Path, out_step: Path, tolerance: str) -> None:
    script = {
        "commands": [
            {
                "command": "file.insert_mesh_solid",
                "params": {"path": str(stl.resolve()), "tolerance": tolerance},
            },
            {
                "command": "file.export",
                "params": {"path": str(out_step.resolve()), "format": "step"},
            },
        ]
    }
    path.write_text(json.dumps(script, indent=2) + "\n", encoding="utf-8")


def convert_case(cli: Path, case: dict[str, Path], out_dir: Path, tolerance: str) -> dict[str, Any]:
    name = case["name"]
    actual = out_dir / f"{name}.actual.step"
    script = out_dir / f"{name}.convert.json"
    results = out_dir / f"{name}.convert.results.json"
    write_convert_script(script, case["stl"], actual, tolerance)
    if actual.exists():
        actual.unlink()
    t0 = time.perf_counter()
    proc = run(
        [
            str(cli),
            "run",
            str(script),
            "--results",
            "--json-out",
            str(results),
        ],
        timeout=900,
    )
    dt = time.perf_counter() - t0
    err = (proc.stderr or "").strip() or (proc.stdout or "").strip()
    ok = proc.returncode == 0 and actual.is_file() and actual.stat().st_size > 0
    detail = ""
    if results.is_file():
        try:
            detail = results.read_text(encoding="utf-8")[:4000]
        except OSError:
            detail = ""
    return {
        "ok": ok,
        "elapsed_s": round(dt, 3),
        "actual_step": actual if ok else None,
        "returncode": proc.returncode,
        "error": None if ok else (err or detail or f"exit {proc.returncode}"),
        "results_path": str(results) if results.is_file() else None,
    }


def eval_step(cli: Path, step: Path, json_out: Path) -> dict[str, Any] | None:
    proc = run([str(cli), "eval", str(step), "--json-out", str(json_out)], timeout=300)
    if proc.returncode != 0 or not json_out.is_file():
        return {
            "ok": False,
            "error": (proc.stderr or proc.stdout or f"eval exit {proc.returncode}").strip()[:2000],
        }
    try:
        data = json.loads(json_out.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        return {"ok": False, "error": f"bad eval JSON: {e}"}
    return {"ok": True, "measure": data}


def entity_histogram(step: Path) -> Counter[str]:
    text = step.read_text(encoding="utf-8", errors="replace")
    counts: Counter[str] = Counter()
    for token in ENTITY_RE.findall(text):
        if token in ENTITY_KEEP:
            counts[token] += 1
    return counts


def volume_of(measure: dict[str, Any] | None) -> float | None:
    if not measure or not measure.get("ok"):
        return None
    m = measure.get("measure") or {}
    total = m.get("total")
    if isinstance(total, dict) and isinstance(total.get("volume_mm3"), (int, float)):
        return float(total["volume_mm3"])
    for key in ("volume_mm3", "volume", "Volume"):
        if key in m and isinstance(m[key], (int, float)):
            return float(m[key])
    bodies = m.get("bodies")
    if isinstance(bodies, list):
        acc = 0.0
        found = False
        for b in bodies:
            if not isinstance(b, dict):
                continue
            for key in ("volume_mm3", "volume"):
                if isinstance(b.get(key), (int, float)):
                    acc += float(b[key])
                    found = True
                    break
        if found:
            return acc
    return None


def area_of(measure: dict[str, Any] | None) -> float | None:
    if not measure or not measure.get("ok"):
        return None
    m = measure.get("measure") or {}
    total = m.get("total")
    if isinstance(total, dict) and isinstance(total.get("area_mm2"), (int, float)):
        return float(total["area_mm2"])
    for key in ("area_mm2", "area", "Area"):
        if key in m and isinstance(m[key], (int, float)):
            return float(m[key])
    bodies = m.get("bodies")
    if isinstance(bodies, list):
        acc = 0.0
        found = False
        for b in bodies:
            if not isinstance(b, dict):
                continue
            for key in ("area_mm2", "area"):
                if isinstance(b.get(key), (int, float)):
                    acc += float(b[key])
                    found = True
                    break
        if found:
            return acc
    return None


def similarity(a: float | None, b: float | None) -> float | None:
    if a is None or b is None:
        return None
    denom = max(abs(a), abs(b), 1e-12)
    return max(0.0, 1.0 - abs(a - b) / denom)


def stl_mesh_volume_mm3(path: Path) -> float | None:
    """Signed volume of a binary STL via the divergence theorem (mm³)."""
    try:
        data = path.read_bytes()
    except OSError:
        return None
    if len(data) < 84:
        return None
    import struct

    n = struct.unpack_from("<I", data, 80)[0]
    if n > 5_000_000 or len(data) < 84 + 50 * n:
        return None
    vol = 0.0
    off = 84
    for _ in range(n):
        vals = struct.unpack_from("<12fH", data, off)
        off += 50
        a, b, c = vals[3:6], vals[6:9], vals[9:12]
        vol += a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])
    return abs(vol) / 6.0


def hist_similarity(a: Counter[str], b: Counter[str]) -> float:
    keys = set(a) | set(b)
    if not keys:
        return 1.0
    scores = []
    for k in keys:
        scores.append(similarity(float(a.get(k, 0)), float(b.get(k, 0))) or 0.0)
    return sum(scores) / len(scores)


def try_compare_step(actual: Path, reference: Path, out_txt: Path) -> dict[str, Any] | None:
    """Optional OCCT xor via the checking skill (separate process; not linked into SolidCraft)."""
    root = Path(__file__).resolve().parents[2]
    compare = root / "skills" / "solidcraft-checking" / "scripts" / "compare_step.py"
    if not compare.is_file():
        return None
    uv = shutil.which("uv")
    if not uv:
        return None
    proc = run(
        [uv, "run", "--no-project", "--with", "build123d", "python", str(compare), str(actual), str(reference)],
        timeout=600,
    )
    text = (proc.stdout or "") + (proc.stderr or "")
    out_txt.write_text(text, encoding="utf-8")
    xor = None
    m = re.search(r"xor\s+([0-9.eE+-]+)\s+mm3", text)
    if m:
        try:
            xor = float(m.group(1))
        except ValueError:
            xor = None
    return {"ok": proc.returncode == 0, "xor_mm3": xor, "log": str(out_txt), "returncode": proc.returncode}


def compare_case(
    cli: Path,
    case: dict[str, Path],
    convert: dict[str, Any],
    out_dir: Path,
    volume_min: float,
) -> dict[str, Any]:
    name = case["name"]
    ref = case["reference"]
    row: dict[str, Any] = {
        "name": name,
        "stl": str(case["stl"]),
        "reference": str(ref),
        "convert_ok": convert["ok"],
        "convert_elapsed_s": convert["elapsed_s"],
        "convert_error": convert.get("error"),
        "pass": False,
    }
    ref_hist = entity_histogram(ref)
    row["reference_entities"] = dict(sorted(ref_hist.items()))
    ref_eval = eval_step(cli, ref, out_dir / f"{name}.reference.eval.json")
    row["reference_volume"] = volume_of(ref_eval)
    row["reference_area"] = area_of(ref_eval)
    row["reference_eval_ok"] = bool(ref_eval and ref_eval.get("ok"))

    if not convert["ok"] or convert.get("actual_step") is None:
        row["reason"] = "SolidCraft convert failed (or wrote no STEP)"
        return row

    actual: Path = convert["actual_step"]
    row["actual"] = str(actual)
    act_hist = entity_histogram(actual)
    row["actual_entities"] = dict(sorted(act_hist.items()))
    row["entity_similarity"] = round(hist_similarity(act_hist, ref_hist), 4)

    act_eval = eval_step(cli, actual, out_dir / f"{name}.actual.eval.json")
    row["actual_volume"] = volume_of(act_eval)
    row["actual_area"] = area_of(act_eval)
    row["actual_eval_ok"] = bool(act_eval and act_eval.get("ok"))
    row["volume_similarity"] = (
        None if similarity(row["actual_volume"], row["reference_volume"]) is None else round(similarity(row["actual_volume"], row["reference_volume"]), 4)
    )
    row["area_similarity"] = (
        None if similarity(row["actual_area"], row["reference_area"]) is None else round(similarity(row["actual_area"], row["reference_area"]), 4)
    )

    occt = try_compare_step(actual, ref, out_dir / f"{name}.compare_step.txt")
    if occt is not None:
        row["occt_xor_mm3"] = occt.get("xor_mm3")
        row["occt_ok"] = occt.get("ok")

    mesh_vol = stl_mesh_volume_mm3(case["stl"])
    row["mesh_volume"] = mesh_vol
    row["mesh_volume_similarity"] = (
        None if similarity(row["actual_volume"], mesh_vol) is None else round(similarity(row["actual_volume"], mesh_vol), 4)
    )

    vol_sim = row["volume_similarity"]
    mesh_sim = row["mesh_volume_similarity"]
    # Pass if we match the reference STEP, or (when the STL is a coarse approximation of that
    # STEP) if we faithfully reconstruct the mesh volume itself.
    if vol_sim is not None and vol_sim >= volume_min:
        row["reason"] = "ok"
        row["pass"] = True
    elif mesh_sim is not None and mesh_sim >= max(volume_min, 0.98):
        row["reason"] = f"ok (mesh volume; ref sim {vol_sim})"
        row["pass"] = True
    elif vol_sim is None and mesh_sim is None:
        row["reason"] = "could not measure volume on actual, reference, or mesh"
        row["pass"] = False
    else:
        row["reason"] = f"volume similarity ref={vol_sim} mesh={mesh_sim} < {volume_min}"
        row["pass"] = False
    return row


def write_report(out_dir: Path, rows: list[dict[str, Any]], volume_min: float) -> Path:
    report = {
        "volume_min": volume_min,
        "cases": rows,
        "passed": sum(1 for r in rows if r.get("pass")),
        "failed": sum(1 for r in rows if not r.get("pass")),
        "total": len(rows),
    }
    json_path = out_dir / "report.json"
    json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    lines = [
        "# STL → STEP A/B report",
        "",
        f"Cases: {report['total']}  passed: {report['passed']}  failed: {report['failed']}",
        f"Volume similarity threshold: {volume_min}",
        "",
        "| case | convert | vol A | vol B | vol sim | mesh sim | entity sim | result |",
        "|---|---|---:|---:|---:|---:|---:|---|",
    ]
    for r in rows:
        lines.append(
            "| {name} | {conv} | {va} | {vb} | {vs} | {ms} | {es} | {res} |".format(
                name=r["name"],
                conv="ok" if r.get("convert_ok") else "FAIL",
                va=_fmt(r.get("actual_volume")),
                vb=_fmt(r.get("reference_volume")),
                vs=_fmt(r.get("volume_similarity")),
                ms=_fmt(r.get("mesh_volume_similarity")),
                es=_fmt(r.get("entity_similarity")),
                res="PASS" if r.get("pass") else f"FAIL ({r.get('reason', '')})",
            )
        )
    lines.append("")
    for r in rows:
        lines.append(f"## {r['name']}")
        lines.append("")
        if r.get("convert_error"):
            lines.append(f"- convert error: `{_one_line(r['convert_error'])}`")
        lines.append(f"- STL: `{r['stl']}`")
        lines.append(f"- reference STEP (B): `{r['reference']}`")
        if r.get("actual"):
            lines.append(f"- SolidCraft STEP (A): `{r['actual']}`")
        if r.get("reference_entities"):
            lines.append(f"- reference entities: `{json.dumps(r['reference_entities'])}`")
        if r.get("actual_entities"):
            lines.append(f"- actual entities: `{json.dumps(r['actual_entities'])}`")
        if r.get("occt_xor_mm3") is not None:
            lines.append(f"- OCCT xor: {r['occt_xor_mm3']} mm³ (ok={r.get('occt_ok')})")
        lines.append("")
    md_path = out_dir / "report.md"
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return md_path


def _fmt(v: Any) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.4g}"
    return str(v)


def _one_line(s: str) -> str:
    return " ".join(s.split())[:240]


def main() -> int:
    here = Path(__file__).resolve().parent
    root = here.parents[1]
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cli", type=Path, help="path to solidcraft-cli.exe")
    parser.add_argument(
        "--fixtures",
        type=Path,
        default=root / "tests" / "stl_to_step_a_b_tests",
        help="directory of name-matched .stl + .step pairs",
    )
    parser.add_argument("--out", type=Path, default=None, help="output directory (default: fixtures/out)")
    parser.add_argument("--case", action="append", default=[], help="run only this basename (repeatable)")
    parser.add_argument("--tolerance", default="0.05 mm", help="mesh.convert / insert_mesh_solid tolerance")
    parser.add_argument("--volume-min", type=float, default=0.95, help="minimum volume similarity to pass")
    args = parser.parse_args()

    fixtures = args.fixtures.resolve()
    out_dir = (args.out or (fixtures / "out")).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    cli = args.cli
    if cli is None:
        for cand in (
            root / "target" / "release" / "solidcraft-cli.exe",
            root / "target" / "debug" / "solidcraft-cli.exe",
            root / "target" / "release" / "solidcraft-cli",
            root / "target" / "debug" / "solidcraft-cli",
        ):
            if cand.is_file():
                cli = cand
                break
    if cli is None or not Path(cli).is_file():
        print("ERROR: solidcraft-cli not found; pass --cli or run build.bat first", file=sys.stderr)
        return 2
    cli = Path(cli).resolve()

    cases = discover_cases(fixtures)
    if args.case:
        want = set(args.case)
        cases = [c for c in cases if c["name"] in want]
        missing = want - {c["name"] for c in cases}
        if missing:
            print(f"ERROR: no fixtures for: {sorted(missing)}", file=sys.stderr)
            return 2
    if not cases:
        print(f"ERROR: no .stl+.step pairs in {fixtures}", file=sys.stderr)
        return 2

    print(f"CLI      : {cli}")
    print(f"fixtures : {fixtures}")
    print(f"out      : {out_dir}")
    print(f"cases    : {len(cases)}")
    print()

    rows: list[dict[str, Any]] = []
    for case in cases:
        print("=" * 72)
        print(f"[case] {case['name']}")
        print(f"[stl]  {case['stl']}")
        print(f"[ref]  {case['reference']}")
        convert = convert_case(cli, case, out_dir, args.tolerance)
        if convert["ok"]:
            print(f"[A]    convert ok in {convert['elapsed_s']}s → {convert['actual_step']}")
        else:
            print(f"[A]    CONVERT FAIL ({convert['elapsed_s']}s): {_one_line(str(convert.get('error')))}")
        row = compare_case(cli, case, convert, out_dir, args.volume_min)
        status = "PASS" if row["pass"] else "FAIL"
        print(
            f"[{status}] vol_sim={row.get('volume_similarity')} "
            f"entity_sim={row.get('entity_similarity')} "
            f"reason={row.get('reason')}"
        )
        rows.append(row)

    report = write_report(out_dir, rows, args.volume_min)
    print()
    print(f"report: {report}")
    failed = sum(1 for r in rows if not r["pass"])
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        raise SystemExit(130)
