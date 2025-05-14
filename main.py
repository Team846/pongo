import os
import re
import json

with open('networktables.json', 'r') as f:
    nt_data = json.load(f)

prefs = {}
for item in nt_data:
    if item['name'].startswith('/Preferences/'):
        key = item['name'][13:]
        prefs[key] = {
            'type': item['type'],
            'value': item['value']  
        }

def find_prefs(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    reg_prefs = []
    pattern = r'RegisterPreference\s*\(\s*"([^"]+)"\s*,\s*([^)]+)\)'
    matches = re.finditer(pattern, content)
    for match in matches:
        key = match.group(1)
        value = match.group(2).strip()
        if value.endswith('_deg'):
            value = float(value.replace('_deg', ''))
        elif value.endswith('_in'):
            value = float(value.replace('_in', ''))
        elif value.endswith('_fps'):
            value = float(value.replace('_fps', ''))
        elif value.endswith('_ms'):
            value = float(value.replace('_ms', ''))
        elif value.endswith('_V'):
            value = float(value.replace('_V', ''))
        elif value.endswith('_A'):
            value = float(value.replace('_A', ''))
        elif value.endswith('_ft'):
            value = float(value.replace('_ft', ''))
        elif value.endswith('_tr'):
            value = float(value.replace('_tr', ''))
        elif value.endswith('_m'):
            value = float(value.replace('_m', ''))
        elif value.endswith('_tps'):
            value = float(value.replace('_tps', ''))
        elif value.endswith('_fps_sq'):
            value = float(value.replace('_fps_sq', ''))
        elif value.endswith('_ms_sq'):
            value = float(value.replace('_ms_sq', ''))
        elif value.lower() == 'true':
            value = True
        elif value.lower() == 'false':
            value = False
        else:
            try:
                value = float(value)
            except ValueError:
                continue
        reg_prefs.append((key, value, file_path))
    return reg_prefs

diffs = []
for root, _, files in os.walk('src/y2025'):
    for file in files:
        if file.endswith(('.cpp', '.cc',)):
            file_path = os.path.join(root, file)
            reg_prefs = find_prefs(file_path)
            for key, value, path in reg_prefs:
                for nt_key, nt_data in prefs.items():
                    if nt_key.endswith('/' + key):
                        if nt_data['value'] != value:
                            diffs.append({
                                'file': path,
                                'key': key,
                                'val': value,
                                'nt_val': nt_data['value']
                            })

for diff in diffs:
    print(f"file: {diff['file']}")
    print(f"key: {diff['key']}")
    print(f"code val: {diff['val']}")
    print(f"nt val: {diff['nt_val']}")
    print()