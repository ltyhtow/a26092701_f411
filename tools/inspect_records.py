import json

with open(r'C:\Users\30496\Desktop\records-2026-10-06-16-45-21.json', 'r', encoding='utf-8') as f:
    records = json.load(f)

print("Total records:", len(records))
for i, r in enumerate(records):
    print(f"Record {i:02d}: time={r.get('time')}, len={r.get('byteLength')}")
