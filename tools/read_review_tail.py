import json

with open(r'C:\Users\30496\.claude\projects\C--Users-30496-CLionProjects-a26092701-cmake\f1a14a4c-24ad-4c44-ad30-a08c2bff5607\subagents\workflows\wf_68b177a2-60b\agent-a5d9b04731056a82b.jsonl', 'r', encoding='utf-8') as f:
    lines = f.readlines()

print("Lines count:", len(lines))
for i in range(-3, 0):
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
