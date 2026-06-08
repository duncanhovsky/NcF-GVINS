#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Evaluate i2Nav ablation batch results and generate a static HTML report."""

import argparse
import datetime as _datetime
import html
import json
import math
import re
import shlex
import statistics
import subprocess
from dataclasses import dataclass
from pathlib import Path


DEFAULT_TRUTH_ROOT = Path("/media/dyishere/RED/dataset/i2Nav-Robot")
DEFAULT_TRAJECTORY_NAMES = ("trajectory.csv", "global_path.csv")
DEFAULT_METRICS = ("trans_part", "angle_deg")
I2NAV_SEQUENCES = (
    "building00",
    "building01",
    "building02",
    "playground00",
    "parking00",
    "street00",
    "street01",
    "street02",
)
STAT_KEYS = ("rmse", "mean", "median", "middle", "min", "max", "std", "sse")
SUMMARY_METRICS = tuple(
    f"ape_{metric}.{stat}"
    for metric in DEFAULT_METRICS
    for stat in ("rmse", "mean", "median", "middle", "min", "max", "std")
)
RUN_DIR_RE = re.compile(r"^run_(\d+)$")
EVO_STAT_RE = re.compile(
    r"^\s*(max|mean|median|min|rmse|sse|std)\s*[:\t ]+\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*$"
)
TUM_ENTRY_COUNT = 8
TUM_DELIMITER_RE = re.compile(r"[\s,]+")


@dataclass(frozen=True)
class RunRecord:
    run_id: str
    batch_id: str
    scheme: str
    sequence: str
    repeat: str
    trajectory_kind: str
    run_dir: Path
    estimate_path: Path
    reference_path: Path
    status: str
    metadata: dict
    status_data: dict


def natural_key(value):
    parts = re.split(r"(\d+)", str(value))
    return tuple(int(part) if part.isdigit() else part for part in parts)


def sanitize_id(value):
    clean = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value))
    return clean.strip("_") or "unknown"


def make_run_id(batch_id, scheme, sequence, repeat, trajectory_kind):
    return "__".join(sanitize_id(part) for part in (batch_id, scheme, sequence, repeat, trajectory_kind))


def trajectory_kind_for_name(filename):
    name = Path(filename).name
    return name[:-4] if name.endswith(".csv") else Path(name).stem


def metric_prefix(metric):
    return f"ape_{metric}"


def parse_key_value_file(path):
    data = {}
    path = Path(path)
    if not path.exists():
        return data
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        data[key.strip()] = value.strip()
    return data


def parse_evo_stats(text):
    stats = {}
    for line in text.splitlines():
        match = EVO_STAT_RE.match(line)
        if not match:
            continue
        key, value = match.groups()
        stats[key] = float(value)
    if "median" in stats:
        stats["middle"] = stats["median"]
    return stats


def truth_path_for_sequence(sequence, truth_root=DEFAULT_TRUTH_ROOT):
    return Path(truth_root) / sequence / f"{sequence}_trajectory.csv"


def infer_run_identity(run_dir, results_root, metadata):
    rel_parts = run_dir.relative_to(results_root).parts
    if len(rel_parts) >= 4:
        batch_id = rel_parts[-4]
        scheme = rel_parts[-3]
        sequence = rel_parts[-2]
    elif len(rel_parts) >= 3:
        batch_id = results_root.name
        scheme = rel_parts[-3]
        sequence = rel_parts[-2]
    else:
        batch_id = results_root.name
        scheme = "unknown_scheme"
        sequence = "unknown_sequence"

    batch_id = metadata.get("batch_id", batch_id)
    scheme = metadata.get("scheme", scheme)
    sequence = metadata.get("sequence", sequence)

    repeat_value = metadata.get("repeat")
    if repeat_value and repeat_value.isdigit():
        repeat = f"run_{int(repeat_value):02d}"
    else:
        repeat = run_dir.name
    return batch_id, scheme, sequence, repeat


def discover_runs(results_root, truth_root=DEFAULT_TRUTH_ROOT, trajectory_names=DEFAULT_TRAJECTORY_NAMES):
    """Discover run directories and their available trajectory files."""
    results_root = Path(results_root)
    truth_root = Path(truth_root)
    trajectory_names = tuple(trajectory_names)
    records = []

    if not results_root.exists():
        return records

    run_dirs = [path for path in results_root.rglob("*") if path.is_dir() and RUN_DIR_RE.match(path.name)]
    for run_dir in sorted(run_dirs, key=lambda path: tuple(natural_key(part) for part in path.parts)):
        metadata = parse_key_value_file(run_dir / "metadata.txt")
        status_data = parse_key_value_file(run_dir / "status.txt")
        status = status_data.get("status", "unknown")
        batch_id, scheme, sequence, repeat = infer_run_identity(run_dir, results_root, metadata)
        reference_path = truth_path_for_sequence(sequence, truth_root)

        for index, trajectory_name in enumerate(trajectory_names):
            estimate_path = run_dir / trajectory_name
            if index > 0 and not estimate_path.exists():
                continue
            trajectory_kind = trajectory_kind_for_name(trajectory_name)
            run_id = make_run_id(batch_id, scheme, sequence, repeat, trajectory_kind)
            records.append(
                RunRecord(
                    run_id=run_id,
                    batch_id=batch_id,
                    scheme=scheme,
                    sequence=sequence,
                    repeat=repeat,
                    trajectory_kind=trajectory_kind,
                    run_dir=run_dir,
                    estimate_path=estimate_path,
                    reference_path=reference_path,
                    status=status,
                    metadata=metadata,
                    status_data=status_data,
                )
            )

    return sorted(records, key=lambda run: (natural_key(run.batch_id), natural_key(run.scheme), natural_key(run.sequence), natural_key(run.repeat), run.trajectory_kind))


def decimate_points(points, max_points):
    if max_points <= 0 or len(points) <= max_points:
        return points
    if max_points == 1:
        return [points[0]]
    step = (len(points) - 1) / float(max_points - 1)
    indices = sorted({round(i * step) for i in range(max_points)})
    return [points[index] for index in indices]


def load_tum_positions(path, max_points=300):
    path = Path(path)
    if not path.exists():
        return []
    points = []
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.replace(",", " ").split()
        if len(parts) < 4:
            continue
        try:
            values = [float(part) for part in parts[:4]]
        except ValueError:
            continue
        points.append([values[1], values[2], values[3]])
    return decimate_points(points, max_points)


def split_tum_line(line):
    return [part for part in TUM_DELIMITER_RE.split(line.strip()) if part]


def format_tum_value(value):
    return f"{value:.9f}"


def prepare_evo_tum_file(input_path, output_path):
    """Write an evo-compatible TUM file with 8 numeric columns and no trailing delimiter."""
    input_path = Path(input_path)
    output_path = Path(output_path)
    report = {
        "input_path": str(input_path),
        "output_path": str(output_path),
        "valid_rows": 0,
        "skipped_rows": 0,
        "bad_rows": 0,
        "bad_row_examples": [],
    }
    normalized_lines = []

    for line_number, raw_line in enumerate(input_path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            report["skipped_rows"] += 1
            continue

        parts = split_tum_line(line)
        if len(parts) != TUM_ENTRY_COUNT:
            report["bad_rows"] += 1
            if len(report["bad_row_examples"]) < 5:
                report["bad_row_examples"].append(
                    {"line": line_number, "entries": len(parts), "text": line[:160]}
                )
            continue

        try:
            values = [float(part) for part in parts]
        except ValueError:
            report["skipped_rows"] += 1
            continue

        if not all(math.isfinite(value) for value in values):
            report["bad_rows"] += 1
            if len(report["bad_row_examples"]) < 5:
                report["bad_row_examples"].append(
                    {"line": line_number, "entries": len(parts), "text": line[:160]}
                )
            continue

        normalized_lines.append(" ".join(format_tum_value(value) for value in values))
        report["valid_rows"] += 1

    if not normalized_lines:
        raise ValueError(f"no valid TUM rows in {input_path}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(normalized_lines) + "\n", encoding="utf-8")
    return report


def attach_tum_format_report(row, label, report):
    prefix = f"tum_format.{label}"
    for key, value in report.items():
        row[f"{prefix}.{key}"] = value


def base_result_row(record):
    row = {
        "run_id": record.run_id,
        "batch_id": record.batch_id,
        "scheme": record.scheme,
        "sequence": record.sequence,
        "repeat": record.repeat,
        "trajectory_kind": record.trajectory_kind,
        "status": record.status,
        "evaluation_status": "pending",
        "error_reason": "",
        "run_dir": str(record.run_dir),
        "estimate_path": str(record.estimate_path),
        "reference_path": str(record.reference_path),
        "estimate_exists": record.estimate_path.exists(),
        "reference_exists": record.reference_path.exists(),
        "evo_traj.command": shlex.join(
            [
                "evo_traj",
                "tum",
                str(record.estimate_path),
                f"--ref={record.reference_path}",
                "-p",
                "--plot_mode=xyz",
                "--align",
                "--correct_scale",
            ]
        ),
    }
    for metric in DEFAULT_METRICS:
        prefix = metric_prefix(metric)
        for stat in STAT_KEYS:
            row[f"{prefix}.{stat}"] = None
        row[f"{prefix}.command"] = ""
        row[f"{prefix}.cache_stdout"] = ""
        row[f"{prefix}.cache_stderr"] = ""
    return row


def evo_command(evo_ape, record, metric, save_results_path=None, reference_path=None, estimate_path=None):
    command = [
        evo_ape,
        "tum",
        str(reference_path if reference_path is not None else record.reference_path),
        str(estimate_path if estimate_path is not None else record.estimate_path),
        "-r",
        metric,
        "-vas",
        "--plot_mode",
        "xyz",
    ]
    if save_results_path is not None:
        command.extend(["--save_results", str(save_results_path)])
    return command


def parse_cached_metric(cache_dir, metric):
    stdout_path = cache_dir / f"ape_{metric}.stdout.txt"
    if not stdout_path.exists():
        return None
    return parse_evo_stats(stdout_path.read_text(encoding="utf-8", errors="replace"))


def write_command_cache(cache_dir, metric, command, stdout, stderr, returncode):
    cache_dir.mkdir(parents=True, exist_ok=True)
    (cache_dir / f"ape_{metric}.command.json").write_text(
        json.dumps({"command": command, "returncode": returncode}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    (cache_dir / f"ape_{metric}.stdout.txt").write_text(stdout or "", encoding="utf-8", errors="replace")
    (cache_dir / f"ape_{metric}.stderr.txt").write_text(stderr or "", encoding="utf-8", errors="replace")


def ensure_text(value):
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def attach_stats(row, metric, stats):
    prefix = metric_prefix(metric)
    for stat, value in stats.items():
        if stat in STAT_KEYS:
            row[f"{prefix}.{stat}"] = value


def add_trajectory_preview(row, record, max_points):
    row["trajectory_preview"] = {
        "estimate": load_tum_positions(record.estimate_path, max_points=max_points),
        "reference": load_tum_positions(record.reference_path, max_points=max_points),
    }


def evaluate_runs(
    runs,
    cache_dir,
    evo_ape="evo_ape",
    skip_evo=False,
    force=False,
    timeout=180,
    max_trajectory_points=300,
    embed_trajectories=True,
    skip_failed_evaluation=False,
):
    """Evaluate discovered runs with evo_ape and return flattened result rows."""
    cache_dir = Path(cache_dir)
    rows = []

    for record in runs:
        row = base_result_row(record)
        record_cache_dir = cache_dir / record.run_id

        if embed_trajectories:
            add_trajectory_preview(row, record, max_points=max_trajectory_points)

        if skip_failed_evaluation and record.status not in ("success", "unknown"):
            row["evaluation_status"] = "skipped_failed_run"
            row["error_reason"] = f"run status is {record.status}"
            rows.append(row)
            continue

        if not record.estimate_path.exists():
            row["evaluation_status"] = "trajectory_missing"
            row["error_reason"] = f"missing estimate trajectory: {record.estimate_path}"
            rows.append(row)
            continue

        if not record.reference_path.exists():
            row["evaluation_status"] = "reference_missing"
            row["error_reason"] = f"missing reference trajectory: {record.reference_path}"
            rows.append(row)
            continue

        if skip_evo:
            loaded_any = False
            for metric in DEFAULT_METRICS:
                cached = parse_cached_metric(record_cache_dir, metric)
                if cached:
                    attach_stats(row, metric, cached)
                    loaded_any = True
            row["evaluation_status"] = "cached" if loaded_any else "not_evaluated"
            rows.append(row)
            continue

        prepared_reference_path = record_cache_dir / "reference.tum"
        prepared_estimate_path = record_cache_dir / f"{record.trajectory_kind}.tum"
        try:
            reference_report = prepare_evo_tum_file(record.reference_path, prepared_reference_path)
            estimate_report = prepare_evo_tum_file(record.estimate_path, prepared_estimate_path)
        except ValueError as exc:
            row["evaluation_status"] = "trajectory_format_invalid"
            row["error_reason"] = str(exc)
            rows.append(row)
            continue

        attach_tum_format_report(row, "reference", reference_report)
        attach_tum_format_report(row, "estimate", estimate_report)

        row["evaluation_status"] = "success"
        for metric in DEFAULT_METRICS:
            prefix = metric_prefix(metric)
            save_results_path = record_cache_dir / f"ape_{metric}.zip"
            record_cache_dir.mkdir(parents=True, exist_ok=True)
            command = evo_command(
                evo_ape,
                record,
                metric,
                save_results_path=save_results_path,
                reference_path=prepared_reference_path,
                estimate_path=prepared_estimate_path,
            )
            row[f"{prefix}.command"] = shlex.join(command)

            cached_stats = None if force else parse_cached_metric(record_cache_dir, metric)
            if cached_stats:
                attach_stats(row, metric, cached_stats)
                continue

            try:
                completed = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    timeout=timeout,
                    check=False,
                )
            except FileNotFoundError:
                row["evaluation_status"] = "tool_missing"
                row["error_reason"] = f"cannot find {evo_ape}"
                break
            except subprocess.TimeoutExpired as exc:
                row["evaluation_status"] = "evo_timeout"
                row["error_reason"] = f"{metric} timed out after {timeout}s"
                write_command_cache(record_cache_dir, metric, command, ensure_text(exc.stdout), ensure_text(exc.stderr), -1)
                break

            write_command_cache(record_cache_dir, metric, command, completed.stdout, completed.stderr, completed.returncode)
            row[f"{prefix}.cache_stdout"] = str(record_cache_dir / f"ape_{metric}.stdout.txt")
            row[f"{prefix}.cache_stderr"] = str(record_cache_dir / f"ape_{metric}.stderr.txt")

            if completed.returncode != 0:
                row["evaluation_status"] = "evo_failed"
                row["error_reason"] = f"{metric} failed with exit code {completed.returncode}"
                break

            stats = parse_evo_stats(completed.stdout + "\n" + completed.stderr)
            if not stats or "rmse" not in stats:
                row["evaluation_status"] = "evo_parse_failed"
                row["error_reason"] = f"cannot parse evo stats for {metric}"
                break
            attach_stats(row, metric, stats)

        rows.append(row)

    return rows


def numeric_values(rows, key):
    values = []
    for row in rows:
        value = row.get(key)
        if isinstance(value, (int, float)) and not math.isnan(value):
            values.append(float(value))
    return values


def aggregate_metric(rows, metric):
    values = numeric_values(rows, metric)
    if not values:
        return {}
    data = {
        f"{metric}.mean": statistics.fmean(values),
        f"{metric}.median": statistics.median(values),
        f"{metric}.middle": statistics.median(values),
        f"{metric}.min": min(values),
        f"{metric}.max": max(values),
        f"{metric}.count": len(values),
    }
    data[f"{metric}.std"] = statistics.pstdev(values) if len(values) > 1 else 0.0
    return data


def group_rows(rows, keys):
    groups = {}
    for row in rows:
        group_key = tuple(row.get(key, "") for key in keys)
        groups.setdefault(group_key, []).append(row)
    return groups


def aggregate_groups(rows, keys):
    output = []
    for group_key, group in sorted(group_rows(rows, keys).items(), key=lambda item: item[0]):
        item = {key: value for key, value in zip(keys, group_key)}
        item["run_count"] = len(group)
        item["success_count"] = sum(1 for row in group if row.get("evaluation_status") in ("success", "cached"))
        item["success_rate"] = item["success_count"] / len(group) if group else 0.0
        for metric in SUMMARY_METRICS:
            item.update(aggregate_metric(group, metric))
        output.append(item)
    return output


def add_baseline_improvements(items, baseline_scheme):
    baseline = {}
    for item in items:
        if item.get("scheme") != baseline_scheme:
            continue
        key = (item.get("sequence"), item.get("trajectory_kind"))
        baseline[key] = item

    for item in items:
        key = (item.get("sequence"), item.get("trajectory_kind"))
        base_item = baseline.get(key)
        if not base_item:
            continue
        for metric in SUMMARY_METRICS:
            metric_mean_key = f"{metric}.mean"
            base_value = base_item.get(metric_mean_key)
            value = item.get(metric_mean_key)
            if isinstance(base_value, (int, float)) and base_value != 0 and isinstance(value, (int, float)):
                item[f"{metric}.improvement_vs_baseline"] = (base_value - value) / base_value


def summarize_results(rows, baseline_scheme="00_ic_baseline"):
    scheme_sequence = aggregate_groups(rows, ("scheme", "sequence", "trajectory_kind"))
    add_baseline_improvements(scheme_sequence, baseline_scheme)

    summary = {
        "overall": {
            "generated_at": _datetime.datetime.now().isoformat(timespec="seconds"),
            "total_rows": len(rows),
            "evaluated_rows": sum(1 for row in rows if row.get("evaluation_status") in ("success", "cached")),
            "missing_reference_rows": sum(1 for row in rows if row.get("evaluation_status") == "reference_missing"),
            "missing_trajectory_rows": sum(1 for row in rows if row.get("evaluation_status") == "trajectory_missing"),
            "schemes": sorted({row.get("scheme", "") for row in rows}, key=natural_key),
            "sequences": sorted({row.get("sequence", "") for row in rows}, key=natural_key),
            "trajectory_kinds": sorted({row.get("trajectory_kind", "") for row in rows}, key=natural_key),
            "baseline_scheme": baseline_scheme,
        },
        "scheme_sequence": scheme_sequence,
        "scheme_overall": aggregate_groups(rows, ("scheme", "trajectory_kind")),
        "sequence_overall": aggregate_groups(rows, ("sequence", "trajectory_kind")),
    }
    return summary


def read_plotly_source(plotly_js):
    if not plotly_js:
        return '<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>'
    source = Path(plotly_js).read_text(encoding="utf-8", errors="replace")
    return f"<script>\n{source}\n</script>"


def safe_json_script(data):
    return json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")


def build_html_report(report_title, rows, summary, plotly_js=None):
    plotly_loader = read_plotly_source(plotly_js)
    rows_json = safe_json_script(rows)
    summary_json = safe_json_script(summary)
    title = html.escape(report_title)
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title}</title>
  {plotly_loader}
  <style>
    :root {{
      color-scheme: light;
      --bg: #f6f7f9;
      --panel: #ffffff;
      --line: #d9dee7;
      --text: #16202a;
      --muted: #5c6875;
      --accent: #0f766e;
      --accent-2: #334155;
      --bad: #b91c1c;
      --good: #047857;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      font-size: 14px;
    }}
    header {{
      padding: 20px 28px 12px;
      border-bottom: 1px solid var(--line);
      background: var(--panel);
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 24px;
      font-weight: 680;
      letter-spacing: 0;
    }}
    main {{ padding: 18px 28px 32px; }}
    .muted {{ color: var(--muted); }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 10px;
      margin-bottom: 16px;
    }}
    .stat, .panel {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
    }}
    .stat {{ padding: 12px; }}
    .stat .value {{ font-size: 22px; font-weight: 700; margin-top: 4px; }}
    .panel {{ padding: 14px; margin-bottom: 16px; }}
    .controls {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 10px;
      align-items: end;
    }}
    label {{ display: grid; gap: 5px; font-size: 12px; color: var(--muted); }}
    select, input {{
      width: 100%;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #fff;
      color: var(--text);
      padding: 8px;
      font-size: 13px;
    }}
    select[multiple] {{ min-height: 86px; }}
    button {{
      border: 1px solid var(--accent);
      border-radius: 6px;
      background: var(--accent);
      color: #fff;
      padding: 9px 12px;
      font-weight: 650;
      cursor: pointer;
    }}
    #chart {{ width: 100%; min-height: 520px; }}
    #recommendation {{ margin-top: 10px; color: var(--accent-2); }}
    table {{ width: 100%; border-collapse: collapse; font-size: 12px; }}
    th, td {{ border-bottom: 1px solid var(--line); padding: 7px 8px; text-align: left; white-space: nowrap; }}
    th {{ position: sticky; top: 0; background: #eef2f7; }}
    .table-wrap {{ max-height: 480px; overflow: auto; border: 1px solid var(--line); border-radius: 6px; }}
    .good {{ color: var(--good); font-weight: 650; }}
    .bad {{ color: var(--bad); font-weight: 650; }}
    @media (max-width: 720px) {{
      header, main {{ padding-left: 14px; padding-right: 14px; }}
      #chart {{ min-height: 420px; }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>{title}</h1>
    <div class="muted">Static evo summary for i2Nav ablation batches. Metrics use evo_ape TUM with <code>-r trans_part</code> and <code>-r angle_deg</code>.</div>
  </header>
  <main>
    <section class="grid" id="summaryCards"></section>
    <section class="panel">
      <div class="controls">
        <label>Metric
          <select id="metricSelect"></select>
        </label>
        <label>Chart
          <select id="chartSelect">
            <option value="auto">Auto recommendation</option>
            <option value="heatmap">Heatmap</option>
            <option value="bar">Grouped bar</option>
            <option value="line">Line</option>
            <option value="box">Box plot</option>
            <option value="violin">Violin</option>
            <option value="histogram">Histogram</option>
            <option value="trajectory">Pose plot</option>
            <option value="table">Comparison table</option>
          </select>
        </label>
        <label>X axis
          <select id="xSelect">
            <option value="sequence">sequence</option>
            <option value="scheme">scheme</option>
            <option value="repeat">repeat</option>
            <option value="trajectory_kind">trajectory_kind</option>
            <option value="batch_id">batch_id</option>
          </select>
        </label>
        <label>Color / group
          <select id="colorSelect">
            <option value="scheme">scheme</option>
            <option value="sequence">sequence</option>
            <option value="trajectory_kind">trajectory_kind</option>
            <option value="batch_id">batch_id</option>
          </select>
        </label>
        <label>Aggregate
          <select id="aggSelect">
            <option value="mean">mean</option>
            <option value="median">median</option>
            <option value="min">min</option>
            <option value="max">max</option>
            <option value="raw">raw</option>
          </select>
        </label>
        <label>Schemes
          <select id="schemeFilter" multiple></select>
        </label>
        <label>Sequences
          <select id="sequenceFilter" multiple></select>
        </label>
        <label>Table metrics
          <select id="tableMetricSelect" multiple></select>
        </label>
        <button id="applyButton" type="button">Update</button>
      </div>
      <div id="recommendation"></div>
    </section>
    <section class="panel">
      <div id="chart"></div>
      <div id="table" class="table-wrap" hidden></div>
    </section>
  </main>
  <script id="results-json" type="application/json">{rows_json}</script>
  <script id="summary-json" type="application/json">{summary_json}</script>
  <script>
    const rows = JSON.parse(document.getElementById('results-json').textContent);
    const summary = JSON.parse(document.getElementById('summary-json').textContent);
    const fields = ['batch_id', 'scheme', 'sequence', 'repeat', 'trajectory_kind'];
    const metricFields = [...new Set(rows.flatMap(row => Object.keys(row).filter(key => key.startsWith('ape_') && typeof row[key] === 'number')))].sort();
    if (!metricFields.length) metricFields.push('ape_trans_part.rmse');

    function uniqueValues(field) {{
      return [...new Set(rows.map(row => row[field]).filter(Boolean))].sort((a, b) => String(a).localeCompare(String(b), undefined, {{numeric: true}}));
    }}

    function fillSelect(id, values, selectedValue) {{
      const el = document.getElementById(id);
      el.innerHTML = '';
      values.forEach(value => {{
        const option = document.createElement('option');
        option.value = value;
        option.textContent = value;
        if (value === selectedValue) option.selected = true;
        el.appendChild(option);
      }});
    }}

    function fillMultiSelect(id, values) {{
      const el = document.getElementById(id);
      el.innerHTML = '';
      values.forEach(value => {{
        const option = document.createElement('option');
        option.value = value;
        option.textContent = value;
        option.selected = true;
        el.appendChild(option);
      }});
    }}

    function selectedValues(id) {{
      return [...document.getElementById(id).selectedOptions].map(option => option.value);
    }}

    function card(label, value, klass='') {{
      return `<div class="stat"><div class="muted">${{label}}</div><div class="value ${{klass}}">${{value}}</div></div>`;
    }}

    function renderSummary() {{
      const overall = summary.overall || {{}};
      const total = overall.total_rows || 0;
      const evaluated = overall.evaluated_rows || 0;
      const missingRef = overall.missing_reference_rows || 0;
      const missingTraj = overall.missing_trajectory_rows || 0;
      document.getElementById('summaryCards').innerHTML = [
        card('Total result rows', total),
        card('Evaluated rows', evaluated, evaluated === total ? 'good' : ''),
        card('Missing references', missingRef, missingRef ? 'bad' : ''),
        card('Missing trajectories', missingTraj, missingTraj ? 'bad' : ''),
        card('Schemes', (overall.schemes || []).length),
        card('Sequences', (overall.sequences || []).length)
      ].join('');
    }}

    function filteredRows() {{
      const schemes = new Set(selectedValues('schemeFilter'));
      const sequences = new Set(selectedValues('sequenceFilter'));
      return rows.filter(row =>
        schemes.has(row.scheme) &&
        sequences.has(row.sequence) &&
        typeof row[document.getElementById('metricSelect').value] === 'number'
      );
    }}

    function aggregate(data, xField, colorField, metric, agg) {{
      if (agg === 'raw') return data;
      const groups = new Map();
      for (const row of data) {{
        const key = `${{row[xField]}}\\u0000${{row[colorField]}}`;
        if (!groups.has(key)) groups.set(key, []);
        groups.get(key).push(row[metric]);
      }}
      const out = [];
      for (const [key, values] of groups.entries()) {{
        const [xValue, colorValue] = key.split('\\u0000');
        const sorted = [...values].sort((a, b) => a - b);
        let value = sorted.reduce((a, b) => a + b, 0) / sorted.length;
        if (agg === 'median') value = sorted[Math.floor(sorted.length / 2)];
        if (agg === 'min') value = sorted[0];
        if (agg === 'max') value = sorted[sorted.length - 1];
        out.push({{[xField]: xValue, [colorField]: colorValue, [metric]: value, count: values.length}});
      }}
      return out;
    }}

    function recommendChart(metric, xField, colorField, agg) {{
      if (metric.includes('rmse') && xField === 'sequence' && colorField === 'scheme') return 'heatmap';
      if (agg === 'raw' && xField === 'repeat') return 'box';
      if (metric.includes('std') || metric.includes('max')) return 'box';
      return 'bar';
    }}

    function recommendationText(chart, metric) {{
      const explain = {{
        heatmap: 'Recommended: heatmap is best for scheme × sequence ablation comparison.',
        bar: 'Recommended: grouped bars make aggregate metric ranking easy to scan.',
        line: 'Recommended: line charts are useful when the X axis has an ordered sequence.',
        box: 'Recommended: box plots show repeat-run stability and outliers.',
        violin: 'Recommended: violin plots show error distribution shape across repeated runs.',
        histogram: 'Recommended: histogram checks the distribution of one metric.',
        trajectory: 'Recommended: pose plot checks qualitative path shape for a selected run.',
        table: 'Recommended: table is best for exact values and sorting outside the report.'
      }};
      return `${{explain[chart] || ''}} Metric: ${{metric}}`;
    }}

    function renderTable(data) {{
      const selectedMetrics = selectedValues('tableMetricSelect');
      const fallbackMetric = document.getElementById('metricSelect').value;
      const metricColumns = selectedMetrics.length ? selectedMetrics : [fallbackMetric];
      const columns = [...fields, 'status', 'evaluation_status', ...metricColumns, 'error_reason'];
      const rowsHtml = data.map(row => `<tr>${{columns.map(col => `<td>${{row[col] ?? ''}}</td>`).join('')}}</tr>`).join('');
      document.getElementById('table').innerHTML = `<table><thead><tr>${{columns.map(col => `<th>${{col}}</th>`).join('')}}</tr></thead><tbody>${{rowsHtml}}</tbody></table>`;
    }}

    function renderStandardChart(data, chart, xField, colorField, metric, agg) {{
      const chartDiv = document.getElementById('chart');
      const tableDiv = document.getElementById('table');
      tableDiv.hidden = chart !== 'table';
      chartDiv.hidden = chart === 'table';
      if (chart === 'table') {{
        renderTable(data);
        return;
      }}
      if (chart === 'histogram') {{
        const groups = uniqueFromData(data, colorField);
        const traces = groups.map(group => ({{
          type: 'histogram',
          name: group,
          x: data.filter(row => row[colorField] === group).map(row => row[metric]),
          opacity: 0.72
        }}));
        Plotly.newPlot(chartDiv, traces, {{barmode: 'overlay', margin: {{t: 28}}, xaxis: {{title: metric}}, yaxis: {{title: 'count'}}}}, {{responsive: true}});
        return;
      }}
      if (chart === 'box' || chart === 'violin') {{
        const groups = uniqueFromData(data, colorField);
        const traces = groups.map(group => ({{
          type: chart,
          name: group,
          y: data.filter(row => row[colorField] === group).map(row => row[metric]),
          box: chart === 'violin' ? {{visible: true}} : undefined,
          meanline: chart === 'violin' ? {{visible: true}} : undefined
        }}));
        Plotly.newPlot(chartDiv, traces, {{margin: {{t: 28}}, yaxis: {{title: metric}}}}, {{responsive: true}});
        return;
      }}
      const aggregated = aggregate(data, xField, colorField, metric, agg);
      const groups = uniqueFromData(aggregated, colorField);
      const traces = groups.map(group => ({{
        type: chart === 'line' ? 'scatter' : 'bar',
        mode: chart === 'line' ? 'lines+markers' : undefined,
        name: group,
        x: aggregated.filter(row => row[colorField] === group).map(row => row[xField]),
        y: aggregated.filter(row => row[colorField] === group).map(row => row[metric])
      }}));
      Plotly.newPlot(chartDiv, traces, {{barmode: 'group', margin: {{t: 28}}, xaxis: {{title: xField}}, yaxis: {{title: metric}}}}, {{responsive: true}});
    }}

    function uniqueFromData(data, field) {{
      return [...new Set(data.map(row => row[field]).filter(value => value !== undefined && value !== null))].sort((a, b) => String(a).localeCompare(String(b), undefined, {{numeric: true}}));
    }}

    function renderHeatmap(data, metric) {{
      const schemes = uniqueFromData(data, 'scheme');
      const sequences = uniqueFromData(data, 'sequence');
      const z = sequences.map(sequence => schemes.map(scheme => {{
        const values = data.filter(row => row.scheme === scheme && row.sequence === sequence).map(row => row[metric]);
        return values.length ? values.reduce((a, b) => a + b, 0) / values.length : null;
      }}));
      Plotly.newPlot('chart', [{{
        type: 'heatmap',
        x: schemes,
        y: sequences,
        z,
        colorscale: 'Viridis',
        hoverongaps: false
      }}], {{margin: {{t: 28}}, xaxis: {{title: 'scheme'}}, yaxis: {{title: 'sequence'}}}}, {{responsive: true}});
    }}

    function renderTrajectory(data) {{
      const row = data.find(item => item.trajectory_preview && item.trajectory_preview.estimate && item.trajectory_preview.estimate.length);
      if (!row) {{
        document.getElementById('chart').innerHTML = '<div class="muted">No embedded trajectory preview for the current filter.</div>';
        return;
      }}
      const estimate = row.trajectory_preview.estimate || [];
      const reference = row.trajectory_preview.reference || [];
      const makeTrace = (name, pts) => ({{
        type: 'scatter3d',
        mode: 'lines',
        name,
        x: pts.map(p => p[0]),
        y: pts.map(p => p[1]),
        z: pts.map(p => p[2])
      }});
      Plotly.newPlot('chart', [makeTrace('estimate', estimate), makeTrace('reference', reference)], {{
        margin: {{t: 28}},
        title: `${{row.scheme}} / ${{row.sequence}} / ${{row.repeat}} / ${{row.trajectory_kind}}`,
        scene: {{xaxis: {{title: 'x'}}, yaxis: {{title: 'y'}}, zaxis: {{title: 'z'}}}}
      }}, {{responsive: true}});
    }}

    function updateChart() {{
      const metric = document.getElementById('metricSelect').value;
      const xField = document.getElementById('xSelect').value;
      const colorField = document.getElementById('colorSelect').value;
      const agg = document.getElementById('aggSelect').value;
      let chart = document.getElementById('chartSelect').value;
      const data = filteredRows();
      if (chart === 'auto') chart = recommendChart(metric, xField, colorField, agg);
      document.getElementById('recommendation').textContent = recommendationText(chart, metric);
      document.getElementById('table').hidden = true;
      document.getElementById('chart').hidden = false;
      if (!data.length) {{
        document.getElementById('chart').innerHTML = '<div class="muted">No rows match the current filters.</div>';
        return;
      }}
      if (chart === 'heatmap') renderHeatmap(data, metric);
      else if (chart === 'trajectory') renderTrajectory(data);
      else renderStandardChart(data, chart, xField, colorField, metric, agg);
    }}

    function init() {{
      renderSummary();
      fillSelect('metricSelect', metricFields, metricFields.includes('ape_trans_part.rmse') ? 'ape_trans_part.rmse' : metricFields[0]);
      fillMultiSelect('schemeFilter', uniqueValues('scheme'));
      fillMultiSelect('sequenceFilter', uniqueValues('sequence'));
      fillMultiSelect('tableMetricSelect', metricFields);
      document.getElementById('applyButton').addEventListener('click', updateChart);
      ['metricSelect', 'chartSelect', 'xSelect', 'colorSelect', 'aggSelect', 'schemeFilter', 'sequenceFilter', 'tableMetricSelect'].forEach(id => {{
        document.getElementById(id).addEventListener('change', updateChart);
      }});
      updateChart();
    }}

    init();
  </script>
</body>
</html>
"""


def write_report(output_dir, rows, summary, report_title="i2Nav ablation evaluation report", plotly_js=None):
    output_dir = Path(output_dir)
    data_dir = output_dir / "data"
    data_dir.mkdir(parents=True, exist_ok=True)

    (data_dir / "results.json").write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
    (data_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    html_text = build_html_report(report_title, rows, summary, plotly_js=plotly_js)
    (output_dir / "index.html").write_text(html_text, encoding="utf-8")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Evaluate i2Nav ablation result folders with evo_ape and generate a static HTML report."
    )
    parser.add_argument("results_root", help="Result folder, e.g. /home/dyishere/ncf_gvins_ws/output/i2nav_ablation_batch")
    parser.add_argument("--truth-root", default=str(DEFAULT_TRUTH_ROOT), help="Root containing <sequence>/<sequence>_trajectory.csv")
    parser.add_argument("--output", default=None, help="Report output directory. Default: <results_root>/i2nav_eval_report")
    parser.add_argument("--cache-dir", default=None, help="evo cache directory. Default: <output>/cache")
    parser.add_argument("--baseline-scheme", default="00_ic_baseline", help="Baseline scheme for improvement ratios")
    parser.add_argument("--evo-ape", default="evo_ape", help="Path to evo_ape executable")
    parser.add_argument("--skip-evo", action="store_true", help="Do not run evo; reuse cached stdout files if present")
    parser.add_argument("--force-evo", action="store_true", help="Re-run evo even when cached stdout exists")
    parser.add_argument("--timeout", type=int, default=180, help="Timeout in seconds for each evo_ape command")
    parser.add_argument("--no-global-path", action="store_true", help="Only evaluate trajectory.csv")
    parser.add_argument("--skip-failed-evaluation", action="store_true", help="Do not attempt evo on runs whose status.txt is not success")
    parser.add_argument("--max-trajectory-points", type=int, default=300, help="Max points embedded per trajectory preview")
    parser.add_argument("--no-embed-trajectories", action="store_true", help="Do not embed downsampled trajectory previews in HTML")
    parser.add_argument("--plotly-js", default=None, help="Optional local plotly.js file to embed for offline reports")
    parser.add_argument("--title", default="i2Nav ablation evaluation report", help="HTML report title")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    results_root = Path(args.results_root)
    truth_root = Path(args.truth_root)
    output_dir = Path(args.output) if args.output else results_root / "i2nav_eval_report"
    cache_dir = Path(args.cache_dir) if args.cache_dir else output_dir / "cache"
    trajectory_names = ("trajectory.csv",) if args.no_global_path else DEFAULT_TRAJECTORY_NAMES

    runs = discover_runs(results_root, truth_root=truth_root, trajectory_names=trajectory_names)
    print(f"[i2Nav evaluator] discovered {len(runs)} trajectory result row(s)")

    rows = evaluate_runs(
        runs,
        cache_dir=cache_dir,
        evo_ape=args.evo_ape,
        skip_evo=args.skip_evo,
        force=args.force_evo,
        timeout=args.timeout,
        max_trajectory_points=args.max_trajectory_points,
        embed_trajectories=not args.no_embed_trajectories,
        skip_failed_evaluation=args.skip_failed_evaluation,
    )
    summary = summarize_results(rows, baseline_scheme=args.baseline_scheme)
    write_report(output_dir, rows, summary, report_title=args.title, plotly_js=args.plotly_js)

    overall = summary["overall"]
    print(f"[i2Nav evaluator] evaluated rows: {overall['evaluated_rows']}/{overall['total_rows']}")
    print(f"[i2Nav evaluator] report: {output_dir / 'index.html'}")
    print(f"[i2Nav evaluator] data: {output_dir / 'data' / 'results.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
