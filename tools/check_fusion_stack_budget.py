"""Check one known IMUFusion call path against its configured task stack.

Read only the three expected GCC .su files in one build's CMakeFiles tree.
This is a known-path budget gate, NOT a complete call-graph/stack-safety proof.
The fixed reserve covers Cortex-M4F task context including alignment (208 bytes), external calls
(512 bytes), and alignment/startup allowance (8 bytes). ISR MSP use, other
paths, recursion, compiler/runtime changes and measured high-water marks still
require separate review. Missing/inlined nodes cannot be assumed to use zero.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import re
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TASK_SOURCE = PROJECT_ROOT / "platform/freertos/src/imu_fusion_task.c"
TASK_MACRO = "IMU_FUSION_TASK_STACK_DEPTH_WORDS"
CONTEXT_BYTES = 208
EXTERNAL_CALL_MARGIN_BYTES = 512
ALIGNMENT_STARTUP_BYTES = 8
RESERVE_BYTES = CONTEXT_BYTES + EXTERNAL_CALL_MARGIN_BYTES + ALIGNMENT_STARTUP_BYTES
SCOPE_NOTE = "Known path only; this is not a complete call-graph or stack-safety proof."

# Exact paths deliberately prevent collecting stale .su files from another
# configuration, host test, nested build, or historical review directory.
SU_FILES = {
    "task": Path("CMakeFiles/imu_freertos.dir/platform/freertos/src/imu_fusion_task.c.su"),
    "fusion": Path("CMakeFiles/imu_fusion_core.dir/middleware/mpu6050/src/imu_fusion.c.su"),
    "vendor": Path("CMakeFiles/fusion_vendor.dir/middleware/mpu6050/third_party/fusion/FusionAhrs.c.su"),
}
KNOWN_CHAIN = (
    ("task", "imu_fusion_task_entry"),
    ("fusion", "imu_fusion_update"),
    ("vendor", "FusionAhrsUpdateNoMagnetometer"),
    ("vendor", "FusionAhrsUpdate"),
)


class BudgetInputError(ValueError):
    """The requested build does not provide an unambiguous static budget."""


@dataclass(frozen=True)
class Entry:
    function: str
    size: int
    qualifier: str


@dataclass(frozen=True)
class Budget:
    words: int
    entries: tuple[Entry, ...]

    @property
    def allocated_bytes(self) -> int:
        return self.words * 4  # This gate targets the STM32F411 Cortex-M4 port.

    @property
    def known_path_bytes(self) -> int:
        return sum(entry.size for entry in self.entries)

    @property
    def required_bytes(self) -> int:
        return self.known_path_bytes + RESERVE_BYTES

    @property
    def passed(self) -> bool:
        return self.required_bytes <= self.allocated_bytes


def read_task_words(source: Path) -> int:
    text = source.read_text(encoding="utf-8-sig")
    text = re.sub(r"/\*[\s\S]*?\*/|//[^\n]*", "", text)
    definitions = re.findall(
        rf"^[ \t]*#[ \t]*define[ \t]+{TASK_MACRO}[ \t]+([^\r\n]+)", text, re.MULTILINE
    )
    if len(definitions) != 1:
        raise BudgetInputError(f"{source}: expected exactly one {TASK_MACRO} definition")
    value = definitions[0].strip()
    match = re.fullmatch(r"(0[xX][0-9a-fA-F]+|0|[1-9][0-9]*)(?:[uU](?:[lL]{1,2})?|[lL]{1,2}[uU]?)?", value)
    if match is None:
        raise BudgetInputError(f"{source}: {TASK_MACRO} must be a positive integer literal, got {value!r}")
    literal = match.group(1)
    words = int(literal, 16 if literal.lower().startswith("0x") else 10)
    if words <= 0:
        raise BudgetInputError(f"{source}: task stack cannot be zero")
    return words


def read_entries(path: Path) -> tuple[Entry, ...]:
    entries = []
    for number, line in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        if not line.strip():
            continue
        parts = line.split("\t")
        # Greedy source field permits Windows drive colons and spaces. GCC C
        # emits source:line:column:function followed by byte count/qualifier.
        location = re.fullmatch(r".+:[0-9]+:[0-9]+:(.+)", parts[0])
        if len(parts) != 3 or location is None or not re.fullmatch(r"[0-9]+", parts[1]):
            raise BudgetInputError(f"{path}:{number}: malformed GCC stack-usage record")
        entries.append(Entry(location.group(1), int(parts[1]), parts[2].strip()))
    if not entries:
        raise BudgetInputError(f"{path}: empty stack-usage file")
    return tuple(entries)


def select_function(entries: tuple[Entry, ...], function: str, path: Path) -> Entry:
    matches = []
    for entry in entries:
        name = entry.function
        clone = re.fullmatch(r"(.+) \[clone (\.[^\]]+)\]", name)
        if clone is not None:
            name = clone.group(1) + clone.group(2)
        if name != function and not name.startswith(function + "."):
            continue
        # Account for common GCC specialization/partition names, but do not
        # silently guess the meaning of an unknown compiler-generated suffix.
        if not re.fullmatch(re.escape(function) + r"(?:\.(?:constprop|isra|part|cold|hot)(?:\.[0-9]+)?)*", name):
            raise BudgetInputError(f"{path}: unsupported suffix for {entry.function}")
        if entry.qualifier != "static":
            raise BudgetInputError(f"{path}: {entry.function} is {entry.qualifier!r}, not static")
        matches.append(entry)
    if not matches:
        raise BudgetInputError(f"{path}: missing {function}; inlining is not evidence of zero stack")
    sizes = {entry.size for entry in matches}
    if len(sizes) != 1:
        detail = ", ".join(f"{entry.function}={entry.size}" for entry in matches)
        raise BudgetInputError(f"{path}: conflicting duplicate/clone records: {detail}")
    return Entry(function, matches[0].size, "static")


def check_budget(build_dir: Path, task_source: Path = DEFAULT_TASK_SOURCE) -> Budget:
    build_dir = build_dir.resolve()
    words = read_task_words(task_source)
    records = {key: read_entries(build_dir / relative) for key, relative in SU_FILES.items()}
    chain = tuple(select_function(records[key], function, build_dir / SU_FILES[key])
                  for key, function in KNOWN_CHAIN)
    return Budget(words, chain)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", required=True, type=Path, help="one firmware CMake build directory")
    parser.add_argument("--task-source", type=Path, default=DEFAULT_TASK_SOURCE,
                        help="source containing the single task-stack word-count definition")
    args = parser.parse_args(argv)
    try:
        budget = check_budget(args.build_dir, args.task_source)
    except (OSError, UnicodeError, BudgetInputError) as exc:
        print(f"FAIL: IMUFusion stack budget cannot be checked: {exc}", file=sys.stderr)
        print(SCOPE_NOTE, file=sys.stderr)
        return 1
    for entry in budget.entries:
        print(f"  {entry.function}: {entry.size} bytes (static)")
    outcome = "PASS" if budget.passed else "FAIL"
    print(f"{outcome}: known path {budget.known_path_bytes} + reserve {RESERVE_BYTES} "
          f"(context {CONTEXT_BYTES}, external calls {EXTERNAL_CALL_MARGIN_BYTES}, "
          f"alignment/startup {ALIGNMENT_STARTUP_BYTES}) = {budget.required_bytes} bytes; "
          f"task allocation {budget.words} words = {budget.allocated_bytes} bytes.")
    print(SCOPE_NOTE)
    return 0 if budget.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
