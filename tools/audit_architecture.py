"""Reproducible inventory for the 2026-10-07 architecture audit; never edits firmware.

Portability labels are reviewed file-level classifications, not an automated proof.
The include graph counts textual includes across all conditional build modes.
"""
from __future__ import annotations
import argparse
from collections import defaultdict
import csv
import hashlib
import json
from pathlib import Path
import re

CLASS_FILES = {
    'portable': ['balance_controller.c', 'safety_fsm.c', 'imu_fusion.c'],
    'rtos': ['balance_task.c', 'imu_task.c', 'imu_fusion_task.c', 'imu_uart_test_task.c'],
    'stm32': ['adc.c', 'app_freertos.c', 'dma.c', 'gpio.c', 'i2c.c', 'main.c',
              'stm32f4xx_hal_msp.c', 'stm32f4xx_hal_timebase_tim.c', 'stm32f4xx_it.c',
              'tim.c', 'usart.c', 'encoder_driver.c', 'encoder_push_test_task.c',
              'motor_driver.c', 'motor_polarity_test_task.c', 'mpu6050_port.c', 'serial_transport.c'],
    'runtime': ['syscalls.c', 'sysmem.c', 'system_stm32f4xx.c'],
}
CLASS_BY_NAME = {name: group for group, names in CLASS_FILES.items() for name in names}
MODULES = ('control', 'encoder', 'motor', 'mpu6050', 'serial')
TOKEN = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*.*?\*/', re.S)

def no_comments(source: str) -> str:
    def replace(match):
        token = match.group()
        if token.startswith('//') or token.startswith('/*'):
            return ''.join('\n' if ch == '\n' else ' ' for ch in token)
        return token
    return TOKEN.sub(replace, source)

def owner(relative: Path) -> str:
    if relative.parts[0] == 'Core': return 'Core'
    if relative.parts[0] == 'middleware': return relative.parts[1]
    if relative.parts[0] == 'app':
        return 'app/diagnostics' if relative.parts[1] == 'diagnostics' else 'app'
    if relative.parts[0] == 'platform':
        return 'platform/ports' if relative.parts[1] == 'include' else 'platform/' + relative.parts[1]
    return relative.parts[0]

def summarize(rows):
    groups = defaultdict(lambda: {'files': 0, 'physical_lines': 0, 'noncomment_lines': 0})
    for row in rows:
        group = groups[row['class']]
        group['files'] += 1
        group['physical_lines'] += row['physical_lines']
        group['noncomment_lines'] += row['noncomment_lines']
    return dict(sorted(groups.items()))

def components(nodes, edges):
    graph = {node: set() for node in nodes}
    for start, end in edges: graph[start].add(end)
    def reach(start):
        seen, stack = set(), [start]
        while stack:
            node = stack.pop()
            if node in seen: continue
            seen.add(node)
            stack.extend(graph[node])
        return seen
    reachable = {node: reach(node) for node in nodes}
    remaining, result = set(nodes), []
    while remaining:
        first = min(remaining)
        group = sorted(node for node in remaining if node in reachable[first] and first in reachable[node])
        remaining.difference_update(group)
        if len(group) > 1: result.append(group)
    return result

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', type=Path, default=Path(__file__).resolve().parent.parent)
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    root = args.root.resolve()
    layered = (root/'middleware'/'control'/'src'/'balance_runtime.c').exists()
    directories = [root/'Core'/'Src', root/'Core'/'Inc']
    module_names = tuple(sorted(directory.name for directory in (root/'middleware').iterdir()
                                if directory.is_dir() and directory.name != 'freertos'))
    for module in module_names:
        directories.extend([root/'middleware'/module/'src', root/'middleware'/module/'include'])
    if layered:
        directories.extend([root/'app'/'src', root/'app'/'include', root/'app'/'diagnostics'/'src',
                            root/'app'/'diagnostics'/'include', root/'platform'/'include',
                            root/'platform'/'stm32f411'/'src', root/'platform'/'stm32f411'/'include',
                            root/'platform'/'freertos'/'src', root/'platform'/'freertos'/'include',
                            root/'platform'/'freertos'/'config'])
    directories = [directory for directory in directories if directory.is_dir()]
    files = sorted(file for directory in directories for file in directory.iterdir()
                   if file.is_file() and file.suffix in ('.c', '.h'))
    headers = {file.name: file for file in files if file.suffix == '.h'}
    if len(headers) != sum(file.suffix == '.h' for file in files):
        raise RuntimeError('Ambiguous project header basename; resolve include search paths explicitly.')
    rows, includes = [], []
    source_hashes = {}
    for file in files:
        relative = file.relative_to(root)
        data = file.read_bytes()
        source_hashes[relative.as_posix()] = hashlib.sha256(data).hexdigest()
        source = data.decode('utf-8-sig')
        cleaned = no_comments(source)
        if file.suffix == '.c':
            if not layered:
                category = CLASS_BY_NAME.get(file.name, 'UNREVIEWED')
            elif relative.parts[0] == 'middleware': category = 'portable'
            elif relative.parts[0] == 'app' or relative.parts[:2] == ('platform', 'freertos'): category = 'rtos'
            elif file.name in CLASS_FILES['runtime']: category = 'runtime'
            else: category = 'stm32'
            rows.append({'file': relative.as_posix(), 'module': owner(relative),
                         'class': category,
                         'physical_lines': len(source.splitlines()),
                         'noncomment_lines': sum(bool(line.strip()) for line in cleaned.splitlines())})
        for line_number, line in enumerate(cleaned.splitlines(), 1):
            match = re.match(r'\s*#\s*include\s*[<"]([^>"]+)[>"]', line)
            target = headers.get(match[1]) if match else None
            if target is None: continue
            other = target.relative_to(root)
            if owner(other) == owner(relative): continue
            includes.append({'from_module': owner(relative), 'to_module': owner(other),
                             'source': relative.as_posix(), 'line': line_number, 'header': other.as_posix()})
    edges = sorted({(entry['from_module'], entry['to_module']) for entry in includes})
    middleware_edges = [(a,b) for a,b in edges if a in module_names and b in module_names]
    nodes = sorted({owner(file.relative_to(root)) for file in files})
    build_db = root/'build'/'static-fix'/'firmware-Debug'/'compile_commands.json'
    build_sources = set()
    if build_db.exists():
        for entry in json.loads(build_db.read_text(encoding='utf-8')):
            file = Path(entry['file']).resolve()
            if file.is_relative_to(root): build_sources.add(file.relative_to(root).as_posix())
    summary = {
        'layout': 'layered' if layered else 'legacy',
        'scope': 'own Core, middleware and (for layered layout) app/platform C/H; no vendor libraries/tools/build files',
        'classification': 'legacy reviewed file labels or current enforced layer ownership; noncomment lines include declarations/preprocessor',
        'c_files': len(rows), 'headers': len(headers),
        'physical_lines': sum(row['physical_lines'] for row in rows),
        'noncomment_lines': sum(row['noncomment_lines'] for row in rows),
        'all_sources': summarize(rows),
        'middleware_sources': summarize([row for row in rows if row['file'].startswith('middleware/')]),
        'per_module': {module: summarize([row for row in rows if row['module'] == module])
                       for module in nodes},
        'cross_module_include_occurrences': len(includes), 'module_edges': edges,
        'middleware_edges': middleware_edges,
        'cyclic_components_all': components(nodes, edges),
        'cyclic_components_middleware': components(module_names, middleware_edges),
        'source_files_in_compile_database': sum(row['file'] in build_sources for row in rows),
        'source_files_absent_from_compile_database': [row['file'] for row in rows if row['file'] not in build_sources],
    }
    requested = args.output or Path('build/architecture-refactor/audit' if layered else 'build/architecture-audit')
    output = requested if requested.is_absolute() else root/requested
    output.mkdir(parents=True, exist_ok=True)
    for name, value in [('metrics.json', summary), ('source_hashes.json', source_hashes), ('include_edges.json', includes)]:
        (output/name).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    with (output/'source_inventory.csv').open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if any(row['class'] == 'UNREVIEWED' for row in rows):
        raise SystemExit('New source files need manual classification before using these totals.')

if __name__ == '__main__':
    main()
