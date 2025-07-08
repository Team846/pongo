import os
import re
import json
import argparse
from typing import Dict, List, Tuple, Optional, Any

"""
see what diffs dont do anytihnng
python main.py --dry-run

python main.py

python main.py --nt-file backup.json --src-dir src/y2025

mode for funny things
python main.py --interactive
"""

#TODO: add colors to text
RED = "\033[91m"
GREEN = "\033[92m"
RESET = "\033[0m"

def read_nt(file_path: str) -> Dict[str, Any]:
    with open(file_path, 'r') as f:
        nt_data = json.load(f)
    
    prefs = {}
    for item in nt_data:
        if item['name'].startswith('/Preferences/'):
            key = item['name'][13:]
            prefs[key] = {
                'value': item['value'],
                'type': item['type'],
                'full_name': item['name']
            }
    return prefs

def get_cntxt(file_path: str) -> Optional[str]:
    """Extract subsystem context from file path and content"""
    file_path_lower = file_path.lower()
    path_mappings = {
        'telescope': ['telescope'],
        'elevator': ['elevator'], 
        'coral_wrist': ['coral_wrist', 'coral/wrist'],
        'coral_end_effector': ['coral_end_effector', 'coral/end_effector', 'coral_ee'],
        'algal_wrist': ['algal_wrist', 'algal/wrist'],
        'algal_end_effector': ['algal_end_effector', 'algal/end_effector', 'algal_ee'],
        'coral_ss': ['coral_ss', 'coral/superstructure'],
        'algal_ss': ['algal_ss', 'algal/superstructure'],
        'SwerveDrivetrain': ['drivetrain', 'swerve'],
        'climber': ['climber'],
        'MotorMonkey': ['motormonkey', 'motor_monkey'],
        'DrivetrainConstructor': ['drivetrain_constructor'],
        'GPD': ['gpd']
    }
    
    for subsystem, patterns in path_mappings.items():
        if any(pattern in file_path_lower for pattern in patterns):
            return subsystem
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        constructor_pattern = r'(\w+)::\1\s*\('
        constructor_matches = re.findall(constructor_pattern, content)
        
        class_to_subsystem = {
            'TelescopeSubsystem': 'telescope',
            'ElevatorSubsystem': 'elevator',
            'DrivetrainSubsystem': 'SwerveDrivetrain',
            'ClimberSubsystem': 'climber',
        }
        
        for match in constructor_matches:
            if match in class_to_subsystem:
                return class_to_subsystem[match]
                
    except Exception:
        pass
    
    return None 

def find_calls(file_path: str) -> List[Dict[str, Any]]:
    reg_prefs = []
    subsystem_context = get_cntxt(file_path)
    
    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
    except Exception as e:
        print(f"error reading {file_path}: {e}")
        return []
    
    patterns = [
        (r'RegisterPreference\s*\(\s*"([^"]+)"\s*,\s*([^)]+)\s*\)', 'direct'),
        (r'REGISTER_MOTOR_CONFIG\s*\(\s*([^,]+)\s*,\s*([^)]+)\s*\)', 'motor_config'),
        (r'REGISTER_PIDF_CONFIG\s*\(\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^)]+)\s*\)', 'pidf_config'),
        (r'REGISTER_SOFTLIMIT_CONFIG\s*\(\s*([^)]+)\s*\)', 'softlimit_config')
    ]
    
    for pattern, pref_type in patterns:
        matches = re.finditer(pattern, content, re.MULTILINE)
        for match in matches:
            line_num = content[:match.start()].count('\n') + 1
            
            try:
                if pref_type == 'motor_config':
                    reg_prefs.extend([
                        {
                            'key': 'motor_configs/current_limit',
                            'fallback': parse_value(match.group(1).strip()),
                            'file_path': file_path,
                            'line_num': line_num,
                            'subsystem': subsystem_context,
                            'match_text': match.group(0)
                        },
                        {
                            'key': 'motor_configs/smart_current_limit',
                            'fallback': parse_value(match.group(2).strip()),
                            'file_path': file_path,
                            'line_num': line_num,
                            'subsystem': subsystem_context,
                            'match_text': match.group(0)
                        }
                    ])
                elif pref_type == 'pidf_config':
                    reg_prefs.extend([
                        {
                            'key': 'gains/_kP',
                            'fallback': parse_value(match.group(1).strip()),
                            'file_path': file_path,
                            'line_num': line_num,
                            'subsystem': subsystem_context,
                            'match_text': match.group(0)
                        },
                        {
                            'key': 'gains/_kI',
                            'fallback': parse_value(match.group(2).strip()),
                            'file_path': file_path,
                            'line_num': line_num,
                            'subsystem': subsystem_context,
                            'match_text': match.group(0)
                        },
                        {
                            'key': 'gains/_kD',
                            'fallback': parse_value(match.group(3).strip()),
                            'file_path': file_path,
                            'line_num': line_num,
                            'subsystem': subsystem_context,
                            'match_text': match.group(0)
                        },
                        {
                            'key': 'gains/_kF',
                            'fallback': parse_value(match.group(4).strip()),
                            'file_path': file_path,
                            'line_num': line_num,
                            'subsystem': subsystem_context,
                            'match_text': match.group(0)
                        }
                    ])
                elif pref_type == 'softlimit_config':
                    reg_prefs.append({
                        'key': 'softlimits',
                        'fallback': parse_value(match.group(1).strip()),
                        'file_path': file_path,
                        'line_num': line_num,
                        'subsystem': subsystem_context,
                        'match_text': match.group(0)
                    })
                else:
                    reg_prefs.append({
                        'key': match.group(1),
                        'fallback': parse_value(match.group(2).strip()),
                        'file_path': file_path,
                        'line_num': line_num,
                        'subsystem': subsystem_context,
                        'match_text': match.group(0)
                    })
            except IndexError as e:
                print(f"warning: pattern matching error in {file_path} line {line_num}: {e}")
                print(f"\tmatch: {match.group(0)}")
                continue
    
    return reg_prefs

def parse_value(value_str: str) -> Any:
    value = value_str.strip()
    
    unit_suffixes = ['_deg', '_m', '_tps', '_fps_sq', '_ms_sq', '_A', '_V', '_in', '_ms', '_fps', '_deg_per_s', '_rad_per_s', '_ohms']
    for suffix in unit_suffixes:
        if value.endswith(suffix):
            value = value.replace(suffix, '')
            break
    
    if value.lower() == 'true':
        return True
    elif value.lower() == 'false':
        return False
    
    try:
        if '.' in value:
            return float(value)
        else:
            return int(value)
    except ValueError:
        return value.strip('"\'')

def find_matching_pref(pref_info: Dict[str, Any], nt_prefs: Dict[str, Any]) -> Tuple[Optional[str], Optional[Any]]:
    key = pref_info['key']
    subsystem = pref_info['subsystem']
    
    if subsystem:
        potential_keys = [
            f"{subsystem}/{key}",
            f"{subsystem}/{key} (A)",
            f"{subsystem}/{key} (V)", 
            f"{subsystem}/{key} (deg)",
            f"{subsystem}/{key} (in)",
            f"{subsystem}/{key} (fps)",
            f"{subsystem}/{key} (ms)",
            f"{subsystem}/{key} (fps_sq)",
            f"{subsystem}/{key} (deg_per_s)",
            f"{subsystem}/{key} (s)",
        ]
        
        for potential_key in potential_keys:
            if potential_key in nt_prefs:
                return potential_key, nt_prefs[potential_key]['value']
    
    candidates = []
    for nt_key in nt_prefs:
        key_base = key.split('/')[-1]
        nt_key_base = nt_key.split('/')[-1]
        
        nt_key_clean = re.sub(r'\s*\([^)]+\)$', '', nt_key_base)
        
        if nt_key_clean == key_base:
            candidates.append((nt_key, nt_prefs[nt_key]['value']))
    
    if subsystem and candidates:
        subsystem_candidates = [(k, v) for k, v in candidates if k.startswith(f"{subsystem}/")]
        if len(subsystem_candidates) == 1:
            return subsystem_candidates[0]
        elif len(subsystem_candidates) > 1:
            print(f"warning: multiple things for {key} in {subsystem}: {[k for k, v in subsystem_candidates]}")
            return subsystem_candidates[0]
    
    if len(candidates) == 1:
        return candidates[0]
    elif len(candidates) > 1:
        print(f"warning: ambiguous preference {key} has {len(candidates)} candidates: {[k for k, v in candidates]}")
        return None, None
    
    return None, None

def update_file(file_path: str, updates: List[Dict[str, Any]]) -> bool:
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        updates.sort(key=lambda x: x['line_num'], reverse=True)
        
        lines = content.split('\n')
        
        for update in updates:
            line_idx = update['line_num'] - 1
            if line_idx < len(lines):
                old_line = lines[line_idx]
                new_value = update['nt_value']
                
                if isinstance(new_value, bool):
                    new_val_str = 'true' if new_value else 'false'
                elif isinstance(new_value, (int, float)):
                    new_val_str = str(new_value)
                else:
                    new_val_str = f'"{new_value}"'
                
                if 'RegisterPreference' in old_line:
                    updated_line = re.sub(
                        r'(RegisterPreference\s*\(\s*"[^"]+"\s*,\s*)[^)]+(\s*\))',
                        f'\\1{new_val_str}\\2',
                        old_line
                    )
                    lines[line_idx] = updated_line
                    print(f"\t\tupdated line {update['line_num']}: {update['key']} = {new_val_str}")
        
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines))
        
        return True
    except Exception as e:
        print(f"error updating {file_path}: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='sync NetworkTables preferences with code fallback values')
    parser.add_argument('--nt-file', default='networktables.json', help='NetworkTables JSON file')
    parser.add_argument('--src-dir', default='src/y2025', help='source directory to scan')
    parser.add_argument('--dry-run', action='store_true', help='show changes without applying them')
    
    args = parser.parse_args()
    
    try:
        nt_prefs = read_nt(args.nt_file)
        print(f"loaded {len(nt_prefs)} preferences from NetworkTables")
    except Exception as e:
        print(f"error loading NetworkTables file: {e}")
        return
    
    all_reg_prefs = []
    
    for root, _, files in os.walk(args.src_dir):
        for file in files:
            if file.endswith(('.cpp', '.cc')):
                file_path = os.path.join(root, file)
                reg_prefs = find_calls(file_path)
                all_reg_prefs.extend(reg_prefs)
    
    print("matching prefs with nt data...")
    all_updates = {}
    unmatched_prefs = []
    
    for pref_info in all_reg_prefs:
        nt_key, nt_value = find_matching_pref(pref_info, nt_prefs)
        
        if nt_key and nt_value != pref_info['fallback']:
            file_path = pref_info['file_path']
            if file_path not in all_updates:
                all_updates[file_path] = []
            
            all_updates[file_path].append({
                'key': pref_info['key'],
                'line_num': pref_info['line_num'],
                'current_fallback': pref_info['fallback'],
                'nt_value': nt_value,
                'full_nt_key': nt_key,
                'subsystem': pref_info['subsystem']
            })
        elif not nt_key:
            unmatched_prefs.append(pref_info)
    
    if unmatched_prefs:
        print(f"\nwarning: {len(unmatched_prefs)} preferences could not be matched:")
        for pref in unmatched_prefs:
            print(f"\t{pref['file_path']}:{pref['line_num']} - {pref['key']} (subsystem: {pref['subsystem']})")
    
    if not all_updates:
        print("no diffs found")
        return
    
    total_changes = sum(len(updates) for updates in all_updates.values())
    print(f"\nfound {total_changes} diffs:\n\n")
    
    for file_path, updates in all_updates.items():
        print(f"\nfile: {file_path}")
        for update in updates:
            print(f"\tline {update['line_num']}: {update['key']}")
            print(f"\t\t current: {update['current_fallback']}")
            print(f"\t\t nt value: {update['nt_value']}")
            print(f"\t\t full nt key: {update['full_nt_key']}")
    
    if args.dry_run:
        print("\nno changes applied")
        return
    
    if not args.dry_run:
        success_count = 0
        total_applied = 0
        for file_path, updates in all_updates.items():
            print(f"\nfile: {file_path}")
            file_updates = []
            for update in updates:
                print(f"\n\t line {update['line_num']}: {update['key']}")
                print(f"\t\t current: {update['current_fallback']}")
                print(f"\t\t nt value: {update['nt_value']}")
                choice = input("\t\t apply this change? [y/N]: ").lower()
                if choice in ['y', 'yes']:
                    file_updates.append(update)
                    total_applied += 1
            if file_updates:
                if update_file(file_path, file_updates):
                    print(f"\tapplied {len(file_updates)} changes to {file_path}")
                    success_count += 1
                else:
                    print(f"\tfailed to update {file_path}")
        print(f"\ndone! applied {total_applied} changes to {success_count} files")
        return

if __name__ == '__main__':
    exit(main())

























































































