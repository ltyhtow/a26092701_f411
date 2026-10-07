import json
from input_path import require_input_path

path = require_input_path("Inspect the last three JSONL entries and any text message content.")
with path.open('r', encoding='utf-8') as f:
    lines = f.readlines()

print("Lines count:", len(lines))
for i in range(-min(3, len(lines)), 0):
    entry = json.loads(lines[i])
    print(f"Index {i}: keys={list(entry.keys())}")
    if 'message' in entry:
        msg = entry['message']
        print(f"  message keys={list(msg.keys())}")
        if 'content' in msg:
            content = msg['content']
            if isinstance(content, list):
                for item in content:
                    print(f"    item type={item.get('type')}")
                    if item.get('type') == 'text':
                        print("    TEXT FOUND:")
                        print(item.get('text'))
