"""Extract strict hex-dump rows, then validate using the normal LwPKT parser."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
from encoder_monitor import StreamParser, CaptureStats

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('input', type=Path)
    ap.add_argument('--output-dir', type=Path, required=True)
    args = ap.parse_args()
    original = args.input.read_bytes()
    content = original.decode('utf-8-sig')
    data = bytearray()
    excluded = []
    rows = 0
    for number, line in enumerate(content.splitlines(), 1):
        if re.fullmatch(r'\s*(?:[0-9a-fA-F]{2})(?:\s+[0-9a-fA-F]{2})*\s*', line):
            data.extend(bytes.fromhex(line))
            rows += 1
        elif line.strip():
            excluded.append({'line': number, 'text': line[:160]})
    parser = StreamParser()
    capture = CaptureStats()
    frames = parser.feed(bytes(data)) + parser.flush_incomplete()
    capture.consume(frames)
    # Repeated-byte patterns are diagnostics, not a replacement decoder.
    separators = [i for i, value in enumerate(data) if value == 0x2d]
    gaps = Counter(b-a for a, b in zip(separators, separators[1:]))
    report = {
        'source': str(args.input), 'sha256': hashlib.sha256(original).hexdigest(),
        'hex_rows': rows, 'byte_count': len(data), 'excluded_lines': excluded,
        'byte_histogram_top': [(f'{b:02X}', n) for b,n in Counter(data).most_common(12)],
        'aa_count': data.count(0xaa), '55_count': data.count(0x55),
        'encoder_header_count': data.count(bytes.fromhex('aa8738')),
        'parser': vars(parser.stats), 'valid_commands': dict(Counter(f'{f.command:02X}' for f in frames)),
        'encoder_frames': capture.encoder_frames,
        'gap_between_2d_top': gaps.most_common(8),
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir/'capture.bin').write_bytes(data)
    (args.output_dir/'analysis.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(capture.summary(parser.stats))

if __name__ == '__main__':
    import sys
    sys.stdout.reconfigure(encoding='utf-8')
    main()
