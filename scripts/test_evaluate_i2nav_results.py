# -*- coding: utf-8 -*-
"""Tests for the i2Nav ablation result evaluator."""

import importlib.util
import json
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path


SCRIPT_PATH = Path(__file__).with_name("evaluate_i2nav_results.py")


def load_module():
    if not SCRIPT_PATH.exists():
        raise AssertionError(f"missing evaluator script: {SCRIPT_PATH}")
    spec = importlib.util.spec_from_file_location("evaluate_i2nav_results", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
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
        repeat_suffix = repeat_name[4:] if repeat_name.startswith("run_") else repeat_name
        (run_dir / "metadata.txt").write_text(
            "\n".join(
                [
                    f"scheme={scheme}",
                    f"sequence={sequence}",
                    f"repeat={repeat_suffix}",
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

    def test_prepare_evo_tum_file_normalizes_spacing_and_trailing_delimiters(self):
        raw_path = self.base / "legacy_trajectory.csv"
        prepared_path = self.base / "cache" / "legacy_trajectory.tum"
        raw_path.write_text(
            "\n".join(
                [
                    "# timestamp tx ty tz qx qy qz qw",
                    "timestamp tx ty tz qx qy qz qw",
                    "0.0     0 0 0 0 0 0 1     ",
                    "1.0,1,2,3,0,0,0,1,",
                ]
            )
            + "\n",
            encoding="utf-8",
        )

        report = self.module.prepare_evo_tum_file(raw_path, prepared_path)

        self.assertEqual(report["valid_rows"], 2)
        self.assertEqual(report["skipped_rows"], 2)
        text = prepared_path.read_text(encoding="utf-8")
        self.assertEqual(
            text,
            "0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000\n"
            "1.000000000 1.000000000 2.000000000 3.000000000 0.000000000 0.000000000 0.000000000 1.000000000\n",
        )
        for line in text.splitlines():
            self.assertEqual(len(line.split(" ")), 8)
            self.assertFalse(line.endswith(" "))

    def test_evaluate_runs_passes_prepared_tum_files_to_evo(self):
        results_root = self.base / "i2nav_ablation_batch"
        batch_root = results_root / "20260605_120000"
        truth_root = self.base / "truth"
        truth_file = truth_root / "parking00" / "parking00_trajectory.csv"
        truth_file.parent.mkdir(parents=True)
        truth_file.write_text(
            "0.0     0 0 0 0 0 0 1     \n1.0     1 0 0 0 0 0 1     \n",
            encoding="utf-8",
        )
        run_dir = self.write_run(batch_root, "00_ic_baseline", "parking00")
        (run_dir / "trajectory.csv").write_text(
            "0.0     0 0 0 0 0 0 1     \n1.0,1,0,0,0,0,0,1,\n",
            encoding="utf-8",
        )
        runs = self.module.discover_runs(results_root, truth_root, trajectory_names=("trajectory.csv",))
        commands = []

        def fake_run(command, **kwargs):
            commands.append(command)
            return type(
                "Completed",
                (),
                {
                    "returncode": 0,
                    "stdout": "rmse 0.100000\nmean 0.100000\nmedian 0.100000\nmin 0.100000\nmax 0.100000\nstd 0.000000\nsse 0.010000\n",
                    "stderr": "",
                },
            )()

        with mock.patch.object(self.module.subprocess, "run", side_effect=fake_run):
            rows = self.module.evaluate_runs(
                runs,
                cache_dir=self.base / "cache",
                force=True,
                embed_trajectories=False,
            )

        self.assertEqual(rows[0]["evaluation_status"], "success")
        self.assertEqual(len(commands), 2)
        self.assertNotEqual(commands[0][2], str(truth_file))
        self.assertNotEqual(commands[0][3], str(run_dir / "trajectory.csv"))
        self.assertTrue(Path(commands[0][2]).exists())
        self.assertTrue(Path(commands[0][3]).exists())
        self.assertEqual(rows[0]["tum_format.estimate.valid_rows"], 2)
        self.assertEqual(rows[0]["tum_format.reference.valid_rows"], 2)
        for path in (Path(commands[0][2]), Path(commands[0][3])):
            for line in path.read_text(encoding="utf-8").splitlines():
                self.assertEqual(len(line.split(" ")), 8)
                self.assertFalse(line.endswith(" "))


if __name__ == "__main__":
    unittest.main()
