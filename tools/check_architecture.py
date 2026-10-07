"""Read-only portable-core and application boundary gate (Python standard library).

Usage: python tools/check_architecture.py [--compile-commands build/.../compile_commands.json]
Vendor FreeRTOS and third_party sources are excluded. This is a conservative
source/include gate, not a replacement for separate-target compilation or review.
Exit status is 0 on success and 1 for violations or unreadable inputs.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys


STANDARD_HEADERS = frozenset(
    "assert.h complex.h ctype.h errno.h fenv.h float.h inttypes.h iso646.h limits.h "
    "locale.h math.h setjmp.h signal.h stdalign.h stdarg.h stdatomic.h stdbit.h "
    "stdbool.h stdckdint.h stddef.h stdint.h stdio.h stdlib.h stdnoreturn.h "
    "string.h tgmath.h threads.h time.h uchar.h wchar.h wctype.h".split()
)
PURE_TARGETS = frozenset(
    "balance_core encoder_core monotonic_clock imu_fusion_core fusion_vendor wheel_test_core "
    "serial_codec protocol_engine lwpkt lwrb".split()
)
LEXEMES = re.compile(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'')
INCLUDE = re.compile(r'^[ \t]*#[ \t]*include[ \t]*([<"])([^>"\n]+)[>"][ \t]*$', re.MULTILINE)
INCLUDE_DIRECTIVE = re.compile(r'^[ \t]*#[ \t]*include\b[^\n]*', re.MULTILINE)
HARDWARE = re.compile(
    r'\b(?:HAL_\w+|LL_\w+|__HAL_\w+|STM32\w*|NVIC_\w+|__NVIC_\w+|'
    r'IRQn_Type|GPIO_TypeDef|TIM_TypeDef|I2C_TypeDef|USART_TypeDef|DMA_TypeDef|'
    r'GPIO_PIN_\w+|GPIO[A-K]|TIM[1-9]\d*|I2C[1-9]\d*|USART[1-9]\d*|UART[1-9]\d*|'
    r'DMA[12]|RCC|FLASH|PWR|SCB|SysTick|CoreDebug|DWT|'
    r'__(?:disable_irq|enable_irq|get_\w+|set_\w+|DMB|DSB|ISB|NOP|WFI|WFE)|'
    r'__asm(?:__)?)\b'
)
RTOS = re.compile(
    r'\b(?:[xv]Task\w*|uxTask\w*|[xv]Queue\w*|uxQueue\w*|[xv]Semaphore\w*|'
    r'[xv]EventGroup\w*|[xv]Timer\w*|pvPort\w*|vPort\w*|'
    r'TaskHandle_t|QueueHandle_t|SemaphoreHandle_t|TickType_t|BaseType_t|UBaseType_t|'
    r'pdMS_TO_TICKS|pdTRUE|pdFALSE|pdPASS|pdFAIL|port[A-Z]\w*|task[A-Z]\w*)\b'
)
WEAK = re.compile(r'\b__weak(?:__)?\b|\b__attribute__\s*\(\([^;{}]*\bweak\b|#\s*pragma\s+weak\b')
RAW_REGISTER = re.compile(
    r'\(\s*(?:volatile\s+)?u?int(?:8|16|32|64)_t\s*\*\s*\)\s*'
    r'0[xX](?:[45][0-9a-fA-F]{7}|[eE]0[0-9a-fA-F]{6})\b'
)
FUSION_TYPE = re.compile(r'\bFusion[A-Z]\w*\b')
FORBIDDEN_HEADER = re.compile(
    r'(^|/)(?:stm32[^/]*|cmsis[^/]*|core_cm[^/]*|FreeRTOS(?:Config)?\.h|'
    r'task\.h|queue\.h|semphr\.h|timers\.h|event_groups\.h|stream_buffer\.h|'
    r'portmacro\.h|portable\.h|projdefs\.h|main\.h|gpio\.h|tim\.h|'
    r'usart\.h|i2c\.h|dma\.h)$', re.IGNORECASE
)


def blank(token: str) -> str:
    return "".join("\n" if char == "\n" else " " for char in token)


def stripped(source: str, strings: bool = False) -> str:
    """Remove comments (and optionally literals), preserving diagnostic lines."""
    return LEXEMES.sub(
        lambda match: blank(match[0]) if strings or match[0].startswith("/") else match[0], source
    )


def owned_middleware(path: Path, root: Path) -> bool:
    try:
        parts = path.relative_to(root).parts
    except ValueError:
        return False
    return len(parts) > 2 and parts[0].lower() == "middleware" and parts[1].lower() != "freertos" and "third_party" not in parts


def source_files(directory: Path):
    return sorted(path for path in directory.rglob("*") if path.suffix.lower() in {".c", ".h"} and path.is_file())


class Gate:
    def __init__(self, root: Path):
        self.root = root
        self.errors: list[str] = []
        self.middleware = [path for path in source_files(root / "middleware") if owned_middleware(path, root)]
        self.application = source_files(root / "App")
        self.header_index: dict[str, list[Path]] = {}
        for directory in ("middleware", "platform", "App", "Core", "Drivers"):
            for path in (root / directory).rglob("*.h"):
                self.header_index.setdefault(path.name.lower(), []).append(path)

    def report(self, path: Path, line: int, rule: str, detail: str):
        try:
            name = path.relative_to(self.root).as_posix()
        except ValueError:
            name = str(path)
        self.errors.append(f"{name}:{line}: [{rule}] {detail}")

    def include_allowed(self, source: Path, header: str, portable: bool) -> bool:
        header = header.replace("\\", "/")
        if header in STANDARD_HEADERS:
            return True
        parts = source.relative_to(self.root).parts
        # The algorithm vendor stays behind its single private implementation.
        if portable and source.relative_to(self.root).as_posix() == "middleware/mpu6050/src/imu_fusion.c":
            if re.fullmatch(r"Fusion[A-Za-z0-9_]+\.h", header):
                return (self.root / "middleware/mpu6050/third_party/fusion" / header).is_file()
        if portable and parts[1] == "serial" and re.fullmatch(r"(?:lwpkt|lwrb)/[A-Za-z0-9_]+\.h", header):
            return bool(self.header_index.get(Path(header).name.lower()))
        if portable and FORBIDDEN_HEADER.search(header):
            return False
        if not portable and re.search(r"(^|/)(?:stm32[^/]*|cmsis[^/]*|core_cm[^/]*|main\.h|gpio\.h|tim\.h|usart\.h|i2c\.h|dma\.h)$", header, re.I):
            return False
        candidate = (source.parent / header).resolve()
        candidates = [candidate] if candidate.is_file() else self.header_index.get(Path(header).name.lower(), [])
        # Do not accept ../platform/main.h just because a similarly named local
        # header exists. A path-bearing include must resolve to that exact suffix.
        if "/" in header and not candidate.is_file():
            normalized = header.lower().lstrip("./")
            candidates = [path for path in candidates if path.as_posix().lower().endswith("/" + normalized)]
        for target in candidates:
            try:
                target_parts = target.relative_to(self.root).parts
            except ValueError:
                continue
            if portable:
                if not owned_middleware(target, self.root):
                    continue
                if parts[1] == "control" and target_parts[1] not in {"control", "contracts"}:
                    continue
                return True
            # Application may compose RTOS adapters and neutral hardware ports.
            # Exactly one composition file may import numeric motor board policy.
            if target_parts[:2] == ("platform", "stm32f411"):
                return source.relative_to(self.root).as_posix() == "App/src/app_freertos.c" and header == "motor_config.h"
            if target_parts[0] in {"Core", "Drivers"} or "third_party" in target_parts:
                continue
            if target_parts[0] in {"App", "platform", "middleware"}:
                return True
        return False

    def inspect(self, path: Path, portable: bool):
        source = path.read_text(encoding="utf-8-sig")
        comments_removed = stripped(source)
        for directive in INCLUDE_DIRECTIVE.finditer(comments_removed):
            match = INCLUDE.fullmatch(directive[0])
            line = comments_removed.count("\n", 0, directive.start()) + 1
            if match is None:
                self.report(path, line, "include", "computed/multiline include is not an auditable literal")
            elif not self.include_allowed(path, match[2], portable):
                self.report(path, line, "include", f"forbidden or unresolved dependency: {match[2]}")
        code = stripped(source, strings=True)
        rules = [("hardware", HARDWARE), ("raw-register", RAW_REGISTER)]
        if portable:
            rules.extend((("rtos", RTOS), ("weak-binding", WEAK)))
        if path.name == "imu_fusion.h" or (portable and path.relative_to(self.root).parts[1] == "control"):
            rules.append(("fusion-private-type", FUSION_TYPE))
        for rule, expression in rules:
            for match in expression.finditer(code):
                self.report(path, code.count("\n", 0, match.start()) + 1, rule, f"platform/private token: {match[0]}")

    def sources(self):
        if not self.middleware or not self.application:
            self.report(self.root, 1, "scope", "expected non-empty middleware and App source trees")
        for path in self.middleware:
            self.inspect(path, portable=True)
        for path in self.application:
            self.inspect(path, portable=False)

    def commands(self, database: Path) -> int:
        entries = json.loads(database.read_text(encoding="utf-8-sig"))
        if not isinstance(entries, list):
            raise ValueError("compile_commands.json must contain a JSON array")
        selected = 0
        seen: set[Path] = set()
        for entry in entries:
            if not isinstance(entry, dict):
                raise ValueError("each compile command must be an object")
            directory = Path(entry["directory"])
            source = Path(entry["file"])
            if not source.is_absolute():
                source = directory / source
            source = source.resolve()
            command = entry.get("command", "")
            arguments = entry.get("arguments")
            if arguments is None:
                # CMake Windows/GCC command strings: keep quoted path fragments
                # joined to -I, without interpreting Windows backslashes as escapes.
                arguments = [token.replace('"', '') for token in re.findall(r'(?:[^\s"]|"[^"]*")+', command)]
            if not isinstance(arguments, list) or not all(isinstance(token, str) for token in arguments):
                raise ValueError("compile command arguments must be an array of strings")
            if not arguments:
                raise ValueError("compile command must have non-empty command or arguments")
            output = (entry.get("output", "") + " " + command + " " + " ".join(arguments)).replace("\\", "/")
            targets = re.findall(r"CMakeFiles/([^/ ]+)\.dir/", output)
            if not owned_middleware(source, self.root) and not any(target in PURE_TARGETS for target in targets):
                continue
            selected += 1
            seen.add(source)
            index = 0
            while index < len(arguments):
                argument = arguments[index]
                include = None
                for prefix in ("-isystem", "-iquote", "-include", "/external:I", "-I", "/I"):
                    if argument == prefix:
                        index += 1
                        if index >= len(arguments):
                            raise ValueError(f"missing path after {prefix}")
                        include = arguments[index]
                        break
                    if argument.startswith(prefix):
                        include = argument[len(prefix):].lstrip("=")
                        break
                if include:
                    include_path = Path(include)
                    if not include_path.is_absolute():
                        include_path = directory / include_path
                    include_path = include_path.resolve()
                    try:
                        components = include_path.relative_to(self.root).parts
                    except ValueError:
                        components = include_path.parts
                    components = [part.lower() for part in components]
                    if any(part in {"core", "drivers", "freertos", "platform", "app", "cmsis"} or
                           part.startswith("stubs") or part.endswith("_stubs") for part in components):
                        self.report(source, 1, "build-include", f"portable compile command exposes {include}")
                index += 1
        for source in self.middleware:
            if source.suffix == ".c" and source.resolve() not in seen:
                self.report(source, 1, "build-coverage", "portable source is absent from compile database")
        if selected == 0:
            self.report(database, 1, "build-coverage", "no portable target commands found")
        return selected


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--compile-commands", type=Path)
    args = parser.parse_args()
    try:
        gate = Gate(args.root.resolve())
        gate.sources()
        count = gate.commands(args.compile_commands.resolve()) if args.compile_commands else None
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"FAIL: architecture gate input error: {error}", file=sys.stderr)
        return 1
    scope = f"{len(gate.middleware)} portable middleware C/H files; {len(gate.application)} App C/H files"
    if count is not None:
        scope += f"; {count} portable compile commands"
    print(f"Checked {scope}.")
    if gate.errors:
        print(f"FAIL: {len(gate.errors)} architecture boundary violation(s):")
        for error in gate.errors:
            print(f"  {error}")
        return 1
    print("PASS: portable middleware, control/contracts, Fusion privacy and App hardware boundaries.")
    if count is None:
        print("Compile include isolation not checked; pass --compile-commands to include that gate.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
