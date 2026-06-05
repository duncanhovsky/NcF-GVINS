# -*- coding: utf-8 -*-
"""Tests for the i2Nav ablation result evaluator."""

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).with_name("evaluate_i2nav_results.py")


def load_module():
    if not SCRIPT_PATH.exists():
        raise AssertionError(f"missing evaluator script: {SCRIPT_PATH}")
    spec = importlib.util.spec_from_file_location("evaluate_i2nav_results", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class EvaluateI2NavResultsTest(unittest.TestCase):
    def setUp(self):
        self.module = load_module()
        self.tmp = tempfile.TemporaryDirectory()
        self.base = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def write_run(self, root, scheme, sequence, repeat_name="run_01", status="success", global_path=False):
        run_dir = root / scheme / sequence / repeat_name
        run_dir.mkdir(parents=True)
        (run_dir / "metadata.txt").write_text(
            "\n".join(
                [
                    f"scheme={scheme}",
                    f"sequence={sequence}",
                    f"repeat={repeat_name.removeprefix('run_')}",
                    f"run_dir={run_dir}",
                ]
            ),
            encoding="utf-8",
        )
        (run_dir / "status.txt").write_text(f"status={status}\n", encoding="utf-8")
        (run_dir / "trajectory.csv").write_text(
            "0.0 0 0 0 0 0 0 1\n1.0 1 0 0 0 0 0 1\n", encoding="utf-8"
        )
        if global_path:
            (run_dir / "global_path.csv").write_text(
                "0.0 0 0 0 0 0 0 1\n1.0 0.9 0 0 0 0 0 1\n", encoding="utf-8"
            )
        return run_dir

    def test_discovers_batch_runs_and_truth_paths(self):
        results_root = self.base / "i2nav_ablation_batch"
        batch_root = results_root / "20260605_120000"
        truth_root = self.base / "truth"
        truth_file = truth_root / "parking00" / "parking00_trajectory.csv"
        truth_file.parent.mkdir(parents=True)
        truth_file.write_text("0.0 0 0 0 0 0 0 1\n", encoding="utf-8")
        self.write_run(batch_root, "00_ic_baseline", "parking00", global_path=True)

        runs = self.module.discover_runs(results_root, truth_root)

        self.assertEqual([run.trajectory_kind for run in runs], ["global_path", "trajectory"])
        self.assertEqual({run.scheme for run in runs}, {"00_ic_baseline"})
        self.assertEqual({run.sequence for run in runs}, {"parking00"})
        self.assertEqual({run.batch_id for run in runs}, {"20260605_120000"})
        self.assertTrue(all(run.reference_path == truth_file for run in runs))

    def test_parse_evo_statistics_with_median_alias(self):
        text = """
        max 2.500000
        mean 0.750000
        median 0.600000
        min 0.100000
        rmse 0.900000
        sse 8.100000
        std 0.300000
        """

        stats = self.module.parse_evo_stats(text)

        self.assertEqual(stats["max"], 2.5)
        self.assertEqual(stats["mean"], 0.75)
        self.assertEqual(stats["median"], 0.6)
        self.assertEqual(stats["middle"], 0.6)
        self.assertEqual(stats["rmse"], 0.9)

    def test_summarizes_results_with_baseline_improvement(self):
        rows = [
            {
                "scheme": "00_ic_baseline",
                "sequence": "parking00",
                "trajectory_kind": "trajectory",
                "status": "success",
                "evaluation_status": "success",
                "ape_trans_part.rmse": 2.0,
                "ape_angle_deg.rmse": 4.0,
            },
            {
                "scheme": "08_nc_full",
                "sequence": "parking00",
                "trajectory_kind": "trajectory",
                "status": "success",
                "evaluation_status": "success",
                "ape_trans_part.rmse": 1.5,
                "ape_angle_deg.rmse": 3.0,
            },
        ]

        summary = self.module.summarize_results(rows, baseline_scheme="00_ic_baseline")

        full = next(item for item in summary["scheme_sequence"] if item["scheme"] == "08_nc_full")
        self.assertAlmostEqual(full["ape_trans_part.rmse.mean"], 1.5)
        self.assertAlmostEqual(full["ape_trans_part.rmse.improvement_vs_baseline"], 0.25)
        self.assertEqual(summary["overall"]["total_rows"], 2)

    def test_write_report_embeds_data_and_writes_json(self):
        output_dir = self.base / "report"
        rows = [
            {
                "run_id": "20260605_120000__08_nc_full__parking00__run_01__trajectory",
                "batch_id": "20260605_120000",
                "scheme": "08_nc_full",
                "sequence": "parking00",
                "repeat": "run_01",
                "trajectory_kind": "trajectory",
                "status": "success",
                "evaluation_status": "success",
                "ape_trans_part.rmse": 1.5,
                "ape_trans_part.mean": 1.0,
                "ape_angle_deg.rmse": 3.0,
                "trajectory_preview": {"estimate": [[0, 0, 0], [1, 1, 0]], "reference": [[0, 0, 0], [1, 1, 0]]},
            }
        ]
        summary = self.module.summarize_results(rows, baseline_scheme="00_ic_baseline")

        self.module.write_report(output_dir, rows, summary, report_title="i2Nav test report")

        html = (output_dir / "index.html").read_text(encoding="utf-8")
        results_json = json.loads((output_dir / "data" / "results.json").read_text(encoding="utf-8"))
        self.assertIn("i2Nav test report", html)
        self.assertIn("Plotly.newPlot", html)
        self.assertIn("ape_trans_part.rmse", html)
        self.assertEqual(results_json[0]["scheme"], "08_nc_full")


if __name__ == "__main__":
    unittest.main()
