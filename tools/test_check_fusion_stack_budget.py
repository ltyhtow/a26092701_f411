"""Regression of the known-path stack gate; inputs are temporary synthetic .su files."""
from contextlib import redirect_stderr, redirect_stdout
import io
from pathlib import Path
import tempfile
import unittest

import check_fusion_stack_budget as gate


class StackBudgetTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.build = self.root / "build"
        self.task_source = self.root / "imu_fusion_task.c"
        self.task_source.write_text(f"#define {gate.TASK_MACRO} 1024U\n", encoding="utf-8")
        self.write_su("task", "imu_fusion_task_entry", 152)
        self.write_su("fusion", "imu_fusion_update", 232)
        self.write_su("vendor", "FusionAhrsUpdateNoMagnetometer", 56)
        self.append_su("vendor", "FusionAhrsUpdate", 2552)

    def record(self, function, size, qualifier="static"):
        return f"C:/Users/Test User/project/source.c:123:9:{function}\t{size}\t{qualifier}\n"

    def path(self, key):
        return self.build / gate.SU_FILES[key]

    def write_su(self, key, function, size, qualifier="static"):
        path = self.path(key)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.record(function, size, qualifier), encoding="utf-8")

    def append_su(self, key, function, size, qualifier="static"):
        with self.path(key).open("a", encoding="utf-8") as output:
            output.write(self.record(function, size, qualifier))

    def check(self):
        return gate.check_budget(self.build, self.task_source)

    def cli(self):
        output, errors = io.StringIO(), io.StringIO()
        with redirect_stdout(output), redirect_stderr(errors):
            result = gate.main(["--build-dir", str(self.build), "--task-source", str(self.task_source)])
        return result, output.getvalue(), errors.getvalue()

    def test_1024_words_pass_known_debug_chain(self):
        budget = self.check()
        self.assertEqual((budget.known_path_bytes, budget.required_bytes, budget.allocated_bytes), (2992, 3720, 4096))
        self.assertTrue(budget.passed)
        result, output, errors = self.cli()
        self.assertEqual(result, 0)
        self.assertIn("not a complete", output)
        self.assertFalse(errors)

    def test_original_512_words_fail(self):
        self.task_source.write_text(f"#define {gate.TASK_MACRO} 512U\n", encoding="utf-8")
        self.assertFalse(self.check().passed)
        result, output, _ = self.cli()
        self.assertEqual(result, 1)
        self.assertIn("2048 bytes", output)
        self.assertIn("FAIL", output)

    def test_missing_file_cannot_fall_back_to_other_build(self):
        self.path("vendor").unlink()
        old = self.build / "historical" / gate.SU_FILES["vendor"]
        old.parent.mkdir(parents=True)
        old.write_text(self.record("FusionAhrsUpdateNoMagnetometer", 56) +
                       self.record("FusionAhrsUpdate", 8), encoding="utf-8")
        self.assertEqual(self.cli()[0], 1)
        with self.assertRaises(OSError):
            self.check()

    def test_missing_function_is_not_assumed_inlined_zero(self):
        self.write_su("vendor", "FusionAhrsUpdate", 2552)
        with self.assertRaisesRegex(gate.BudgetInputError, "missing FusionAhrsUpdateNoMagnetometer"):
            self.check()

    def test_dynamic_and_bounded_dynamic_are_rejected(self):
        for qualifier in ("dynamic", "dynamic,bounded", "unknown", ""):
            with self.subTest(qualifier=qualifier):
                self.write_su("fusion", "imu_fusion_update", 232, qualifier)
                with self.assertRaisesRegex(gate.BudgetInputError, "not static"):
                    self.check()

    def test_empty_and_malformed_input_fail(self):
        for content in ("", "  \n", "malformed\n", "file.c:1:2:imu_fusion_update\tNaN\tstatic\n"):
            with self.subTest(content=content):
                self.path("fusion").write_text(content, encoding="utf-8")
                self.assertEqual(self.cli()[0], 1)

    def test_known_gcc_clone_suffixes(self):
        for name in ("imu_fusion_update.constprop.0", "imu_fusion_update.isra.1.constprop.2",
                     "imu_fusion_update [clone .isra.0]", "imu_fusion_update.constprop"):
            with self.subTest(name=name):
                self.write_su("fusion", name, 232)
                self.assertEqual(self.check().known_path_bytes, 2992)

    def test_conflicting_duplicates_fail_including_original_and_clone(self):
        for name in ("imu_fusion_update", "imu_fusion_update.constprop.0"):
            with self.subTest(name=name):
                self.write_su("fusion", "imu_fusion_update", 232)
                self.append_su("fusion", name, 200)
                with self.assertRaisesRegex(gate.BudgetInputError, "conflicting"):
                    self.check()

    def test_identical_duplicate_sizes_do_not_double_count(self):
        self.append_su("fusion", "imu_fusion_update.constprop.0", 232)
        self.assertEqual(self.check().known_path_bytes, 2992)

    def test_unknown_suffix_does_not_hide_behind_original(self):
        self.append_su("fusion", "imu_fusion_update.unknown.1", 232)
        with self.assertRaisesRegex(gate.BudgetInputError, "unsupported suffix"):
            self.check()

    def test_macro_must_be_single_positive_literal(self):
        for content in ("", f"#define {gate.TASK_MACRO} 0U\n",
                        f"#define {gate.TASK_MACRO} (1024U)\n",
                        f"#define {gate.TASK_MACRO} 01024U\n",
                        f"#define {gate.TASK_MACRO} 512U * 2U\n",
                        f"#define {gate.TASK_MACRO} 1024U\n#define {gate.TASK_MACRO} 1024U\n"):
            with self.subTest(content=content):
                self.task_source.write_text(content, encoding="utf-8")
                with self.assertRaises(gate.BudgetInputError):
                    self.check()

    def test_comments_and_windows_line_endings(self):
        self.task_source.write_bytes((f"/* #define {gate.TASK_MACRO} 512U */\r\n"
                                      f"#define {gate.TASK_MACRO} 0x400UL // words\r\n").encode())
        self.assertEqual(self.check().words, 1024)


if __name__ == "__main__":
    unittest.main()
