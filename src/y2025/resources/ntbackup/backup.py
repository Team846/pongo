import os
import re
import json
import argparse
from typing import Dict, List, Tuple, Optional, Any

def read_nt(file_path: str) -> Dict[str, Any]:
    with open(file_path, 'r') as f:
        nt_data = json.load(f)
    return {item['name'][13:]: {'value': item['value'], 'type': item['type'], 'full_name': item['name']} 
            for item in nt_data if item['name'].startswith('/Preferences/')}

def get_cntxt(file_path: str) -> Optional[str]:
    path_mappings = {
        'telescope': ['telescope'], 'elevator': ['elevator'], 'coral_wrist': ['coral_wrist', 'coral/wrist'],
        'coral_end_effector': ['coral_end_effector', 'coral/end_effector', 'coral_ee'],
        'algal_wrist': ['algal_wrist', 'algal/wrist'], 'algal_end_effector': ['algal_end_effector', 'algal/end_effector', 'algal_ee'],
        'coral_ss': ['coral_ss', 'coral/superstructure'], 'algal_ss': ['algal_ss', 'algal/superstructure'],
        'SwerveDrivetrain': ['drivetrain', 'swerve'], 'climber': ['climber'], 'MotorMonkey': ['motormonkey', 'motor_monkey'],
        'DrivetrainConstructor': ['drivetrain_constructor'], 'GPD': ['gpd']
    }
    
    file_path_lower = file_path.lower()
    for subsystem, patterns in path_mappings.items():
        if any(pattern in file_path_lower for pattern in patterns):
            return subsystem
    
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        constructor_matches = re.findall(r'(\w+)::\1\s*\(', content)
        class_to_subsystem = {'TelescopeSubsystem': 'telescope', 'ElevatorSubsystem': 'elevator', 'DrivetrainSubsystem': 'SwerveDrivetrain', 'ClimberSubsystem': 'climber'}
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
        for match in re.finditer(pattern, content, re.MULTILINE):
            line_num = content[:match.start()].count('\n') + 1
            try:
                base_info = {'file_path': file_path, 'line_num': line_num, 'subsystem': subsystem_context, 'match_text': match.group(0)}
                
                if pref_type == 'motor_config':
                    reg_prefs.extend([
                        {**base_info, 'key': 'motor_configs/current_limit', 'fallback': parse_value(match.group(1).strip())},
                        {**base_info, 'key': 'motor_configs/smart_current_limit', 'fallback': parse_value(match.group(2).strip())}
                    ])
                elif pref_type == 'pidf_config':
                    keys = ['gains/_kP', 'gains/_kI', 'gains/_kD', 'gains/_kF']
                    reg_prefs.extend([{**base_info, 'key': keys[i], 'fallback': parse_value(match.group(i+1).strip())} for i in range(4)])
                elif pref_type == 'softlimit_config':
                    reg_prefs.append({**base_info, 'key': 'softlimits', 'fallback': parse_value(match.group(1).strip())})
                else:
                    reg_prefs.append({**base_info, 'key': match.group(1), 'fallback': parse_value(match.group(2).strip())})
            except IndexError as e:
                print(f"warning: pattern matching error in {file_path} line {line_num}: {e}")
                continue
    
    return reg_prefs

def parse_value(value_str: str) -> Any:
    value = value_str.strip()
    for suffix in ['_deg', '_m', '_tps', '_fps_sq', '_ms_sq', '_A', '_V', '_in', '_ms', '_fps', '_deg_per_s', '_rad_per_s', '_ohms']:
        if value.endswith(suffix):
            value = value.replace(suffix, '')
            break
    
    if value.lower() == 'true': return True
    if value.lower() == 'false': return False
    
    try:
        return float(value) if '.' in value else int(value)
    except ValueError:
        return value.strip('"\'')

def find_pref(pref_info: Dict[str, Any], nt_prefs: Dict[str, Any]) -> Tuple[Optional[str], Optional[Any]]:
    key, subsystem = pref_info['key'], pref_info['subsystem']
    
    if subsystem:
        units = ['', ' (A)', ' (V)', ' (deg)', ' (in)', ' (fps)', ' (ms)', ' (fps_sq)', ' (deg_per_s)', ' (s)']
        for unit in units:
            potential_key = f"{subsystem}/{key}{unit}"
            if potential_key in nt_prefs:
                return potential_key, nt_prefs[potential_key]['value']
    
    candidates = []
    key_base = key.split('/')[-1]
    for nt_key in nt_prefs:
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

def extrt_val(value_str):
    if isinstance(value_str, (int, float)): return float(value_str)
    match = re.search(r'([+-]?\d*\.?\d+)(?:[_a-zA-Z].*)?', str(value_str))
    return float(match.group(1)) if match else None

def get_unit(nt_key):
    match = re.search(r'\(([^)]+)\)', nt_key)
    if match:
        unit = match.group(1)
        unit_map = {'in': '_in', 'deg': '_deg', 'A': '_A', 'V': '_V', 'Nm': '_Nm', 'fps': '_fps', 'deg_per_s': '_deg_per_s', 'tps': '_tps', 'ms': '_ms', 's': '_s'}
        return unit_map.get(unit, f'_{unit}')
    return None

def norm_val(nt_value, current_value, nt_key):
    current_numeric = extrt_val(current_value)
    nt_numeric = extrt_val(nt_value)
    
    if current_numeric is None or nt_numeric is None or abs(current_numeric - nt_numeric) < 1e-6:
        return None
    
    new_value = str(nt_value)
    unit_suffix = re.search(r'([+-]?\d*\.?\d+)([_a-zA-Z][a-zA-Z0-9_]*)', str(current_value))
    unit_suffix = unit_suffix.group(2) if unit_suffix else get_unit(nt_key)
    
    return f"{nt_value}{unit_suffix}" if unit_suffix else new_value

def create_pat(key, current_value, line_content):
    if 'REGISTER_PIDF_CONFIG' in line_content and 'gains/' in key:
        param_map = {'gains/_kP': 0, 'gains/_kI': 1, 'gains/_kD': 2, 'gains/_kF': 3}
        if key in param_map:
            return rf'(REGISTER_PIDF_CONFIG\s*\(\s*)([^,]+)(\s*,\s*)([^,]+)(\s*,\s*)([^,]+)(\s*,\s*)([^)]+)(\s*\);?)', param_map[key]
    
    if 'RegisterPreference' in line_content:
        return rf'(RegisterPreference\s*\(\s*"{re.escape(key)}"\s*,\s*)([^;)]+)(\s*\);?)'
    elif '=' in line_content and key in line_content:
        return rf'(\b{re.escape(key)}\s*=\s*)([^,;)}}]+)'
    elif '.' + key in line_content:
        return rf'(\.{re.escape(key)}\s*=\s*)([^,;)}}]+)'
    else:
        return rf'(\b{re.escape(key)}\s*[=,]\s*)([^,;)}}]+)'

def apply_pidf(line, param_index, new_value):
    pattern = r'(REGISTER_PIDF_CONFIG\s*\(\s*)([^,]+)(\s*,\s*)([^,]+)(\s*,\s*)([^,]+)(\s*,\s*)([^)]+)(\s*\);?)'
    match = re.search(pattern, line)
    if not match: return line
    
    params = [match.group(2).strip(), match.group(4).strip(), match.group(6).strip(), match.group(8).strip()]
    params[param_index] = str(new_value)
    
    return f"{match.group(1)}{params[0]}{match.group(3)}{params[1]}{match.group(5)}{params[2]}{match.group(7)}{params[3]}{match.group(9)}"


def apply_updates(file_path: str, updates: List[Dict[str, Any]]) -> bool:
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()
        
        updates.sort(key=lambda x: x['line_num'], reverse=True)
        applied_count = 0
        
        for update in updates:
            line_idx = update['line_num'] - 1
            if line_idx < 0 or line_idx >= len(lines):
                print(f"\twarning: line {update['line_num']} out of range")
                continue
                
            original_line = lines[line_idx].rstrip('\n')
            normalized_value = norm_val(update['nt_value'], update['current_fallback'], update['full_nt_key'])
            
            if normalized_value is None:
                print(f"\tskipping line {update['line_num']}: {update['key']} (values equivalent)")
                continue
            
            pattern_result = create_pat(update['key'], update['current_fallback'], original_line)
            
            if isinstance(pattern_result, tuple):
                pattern, param_index = pattern_result
                new_line = apply_pidf(original_line, param_index, normalized_value)
                if new_line != original_line:
                    lines[line_idx] = new_line + '\n'
                    applied_count += 1
                    print(f"\tupdated line {update['line_num']}: {update['key']} = {normalized_value}")
                else:
                    print(f"\twarning: could not update PIDF line {update['line_num']}: {update['key']}")
            else:
                replacement = rf'\g<1>{normalized_value}\g<3>' if 'RegisterPreference' in original_line else rf'\g<1>{normalized_value}'
                try:
                    new_line = re.sub(pattern_result, replacement, original_line)
                    if new_line != original_line:
                        lines[line_idx] = new_line + '\n'
                        applied_count += 1
                        print(f"\tupdated line {update['line_num']}: {update['key']} = {normalized_value}")
                    else:
                        print(f"\twarning: could not match pattern for line {update['line_num']}: {update['key']}")
                except re.error as e:
                    print(f"\tregex error on line {update['line_num']}: {e}")
                    continue
        
        with open(file_path, 'w') as file:
            file.writelines(lines)
        
        print(f"\tapplied {applied_count} changes to {file_path}")
        return True
        
    except Exception as e:
        print(f"error updating {file_path}: {e}")
        return False

def load_ambiguous_resolutions(resolution_file='res.json'):
    if os.path.exists(resolution_file):
        try:
            with open(resolution_file, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Warning: Could not load resolution file {resolution_file}: {e}")
    return {}

def save_ambiguous_resolutions(resolutions, resolution_file='res.json'):
    try:
        with open(resolution_file, 'w') as f:
            json.dump(resolutions, f, indent=2, sort_keys=True)
        print(f"Saved resolutions to {resolution_file}")
    except Exception as e:
        print(f"Warning: Could not save resolution file {resolution_file}: {e}")

def create_resolution_key(pref_info):
    return f"{os.path.relpath(pref_info['file_path'])}:{pref_info['key']}"

def resolve_ambiguous_preferences(ambiguous_prefs, resolution_file='res.json'):
    stored_resolutions = load_ambiguous_resolutions(resolution_file)
    resolved_prefs = {}
    new_resolutions = {}
    
    print(f"\nFound {len(ambiguous_prefs)} ambiguous preferences that need resolution:")
    if stored_resolutions:
        print(f"Loaded {len(stored_resolutions)} previously saved resolutions from {resolution_file}")
    
    for key_id, data in ambiguous_prefs.items():
        pref_info = data['pref_info']
        candidates = data['candidates']
        resolution_key = create_resolution_key(pref_info)
        if resolution_key in stored_resolutions:
            selected_nt_key = stored_resolutions[resolution_key]
            
            if selected_nt_key == "__SKIP__":
                print(f"Using stored resolution: SKIP {pref_info['key']} in {os.path.relpath(pref_info['file_path'])}")
                continue
            
            selected_value = next((v for k, v in candidates if k == selected_nt_key), None)
            
            if selected_value is not None:
                resolved_prefs[key_id] = {'pref_info': pref_info, 'nt_key': selected_nt_key, 'nt_value': selected_value}
                print(f"Using stored resolution: {pref_info['key']} -> {selected_nt_key} = {selected_value}")
                continue
            else:
                print(f"Warning: Stored resolution for {pref_info['key']} references non-existent key {selected_nt_key}")
        print(f"\nambiguous prefs: {pref_info['key']}")
        print(f"Found in: {os.path.relpath(pref_info['file_path'])}")
        print(f"Subsystem: {pref_info['subsystem']}")
        print(f"Current fallback: {pref_info['fallback']}")
        
        print(f"\nCandidates ({len(candidates)} found):")
        for i, (nt_key, nt_value) in enumerate(candidates, 1):
            print(f"  {i}. {nt_key} = {nt_value}")
        print(f"  0. Skip this preference (no update)")
        
        while True:
            try:
                choice = input(f"\nChoose candidate (0-{len(candidates)}): ").strip()
                choice_num = int(choice)
                
                if choice_num == 0:
                    print("Skipping this preference.")
                    new_resolutions[resolution_key] = "__SKIP__"
                    break
                elif 1 <= choice_num <= len(candidates):
                    selected_key, selected_value = candidates[choice_num - 1]
                    resolved_prefs[key_id] = {'pref_info': pref_info, 'nt_key': selected_key, 'nt_value': selected_value}
                    print(f"Selected: {selected_key} = {selected_value}")
                    new_resolutions[resolution_key] = selected_key
                    break
                else:
                    print(f"Invalid choice. Please enter 0-{len(candidates)}")
            except ValueError:
                print("Invalid input. Please enter a number.")
    if new_resolutions:
        all_resolutions = {**stored_resolutions, **new_resolutions}
        save_ambiguous_resolutions(all_resolutions, resolution_file)
        print(f"Saved {len(new_resolutions)} new resolution(s) to {resolution_file}")
    
    return resolved_prefs

def find_pref_with_ambiguity_tracking(pref_info: Dict[str, Any], nt_prefs: Dict[str, Any]) -> Tuple[Optional[str], Optional[Any], List[Tuple[str, Any]]]:
    key, subsystem = pref_info['key'], pref_info['subsystem']
    if subsystem:
        units = ['', ' (A)', ' (V)', ' (deg)', ' (in)', ' (fps)', ' (ms)', ' (fps_sq)', ' (deg_per_s)', ' (s)', ' (Nm)', ' (tps)']
        for unit in units:
            potential_key = f"{subsystem}/{key}{unit}"
            if potential_key in nt_prefs:
                return potential_key, nt_prefs[potential_key]['value'], []
    
    candidates = []
    key_base = key.split('/')[-1]
    for nt_key in nt_prefs:
        nt_key_base = nt_key.split('/')[-1]
        nt_key_clean = re.sub(r'\s*\([^)]+\)$', '', nt_key_base)
        if nt_key_clean == key_base:
            candidates.append((nt_key, nt_prefs[nt_key]['value']))
    if subsystem and candidates:
        subsystem_candidates = [(k, v) for k, v in candidates if k.startswith(f"{subsystem}/")]
        if len(subsystem_candidates) == 1:
            return subsystem_candidates[0][0], subsystem_candidates[0][1], []
        elif len(subsystem_candidates) > 1:
            return None, None, subsystem_candidates
    
    return (candidates[0][0], candidates[0][1], []) if len(candidates) == 1 else (None, None, candidates if len(candidates) > 1 else [])

def main():
    parser = argparse.ArgumentParser(description='sync nt prefs with code fallback values')
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_nt_file = os.path.join(script_dir, '../../../../networktables.json')
    default_src_dir = os.path.join(script_dir, '../../')
    
    parser.add_argument('--nt-file', default=default_nt_file, help='nt file')
    parser.add_argument('--src-dir', default=default_src_dir, help='src dir')
    parser.add_argument('--dry-run', action='store_true', help='dont do anything just show diffs')
    parser.add_argument('--resolve', action='store_true', help='resolve ambig prefs')    
    args = parser.parse_args()
    
    try:
        nt_prefs = read_nt(args.nt_file)
        print(f"loaded {len(nt_prefs)} preferences from NetworkTables")
    except Exception as e:
        print(f"error loading NetworkTables file: {e}")
        print(f"Tried to load from: {os.path.abspath(args.nt_file)}")
        return
    
    all_reg_prefs = []
    for root, _, files in os.walk(args.src_dir):
        for file in files:
            if file.endswith(('.cpp', '.cc')):
                all_reg_prefs.extend(find_calls(os.path.join(root, file)))
    
    print("matching prefs with nt data...")
    all_updates = {}
    unmatched_prefs = []
    ambiguous_prefs = {}
    
    for pref_info in all_reg_prefs:
        nt_key, nt_value, candidates = find_pref_with_ambiguity_tracking(pref_info, nt_prefs)
        
        if nt_key and nt_value != pref_info['fallback']:
            if pref_info['file_path'] not in all_updates:
                all_updates[pref_info['file_path']] = []
            all_updates[pref_info['file_path']].append({
                'key': pref_info['key'], 'line_num': pref_info['line_num'],
                'current_fallback': pref_info['fallback'], 'nt_value': nt_value,
                'full_nt_key': nt_key, 'subsystem': pref_info['subsystem']
            })
        elif candidates:
            key_id = f"{pref_info['file_path']}:{pref_info['line_num']}:{pref_info['key']}"
            ambiguous_prefs[key_id] = {'pref_info': pref_info, 'candidates': candidates}
            print(f"warning: ambiguous preference {pref_info['key']} has {len(candidates)} candidates: {[k for k, v in candidates]}")
        elif not nt_key:
            unmatched_prefs.append(pref_info)

    if ambiguous_prefs and args.resolve:
        resolved_prefs = resolve_ambiguous_preferences(ambiguous_prefs)
        
        for key_id, resolved_data in resolved_prefs.items():
            pref_info = resolved_data['pref_info']
            nt_key = resolved_data['nt_key']
            nt_value = resolved_data['nt_value']
            
            if nt_value != pref_info['fallback']:
                if pref_info['file_path'] not in all_updates:
                    all_updates[pref_info['file_path']] = []
                all_updates[pref_info['file_path']].append({
                    'key': pref_info['key'], 'line_num': pref_info['line_num'],
                    'current_fallback': pref_info['fallback'], 'nt_value': nt_value,
                    'full_nt_key': nt_key, 'subsystem': pref_info['subsystem']
                })
        
        print(f"\nResolved {len(resolved_prefs)} ambiguous preferences.")
    
    if unmatched_prefs:
        print(f"\nwarning: {len(unmatched_prefs)} preferences could not be matched:")
        for pref in unmatched_prefs:
            print(f"\t{pref['file_path']}:{pref['line_num']} - {pref['key']} (subsystem: {pref['subsystem']})")
    
    if not all_updates:
        print("no differences found")
        return
    
    total_diffs = sum(len(updates) for updates in all_updates.values())
    print(f"found {total_diffs} diffs:\n")
    
    for file_path, updates in all_updates.items():
        print(f"file: {file_path}")
        for update in updates:
            print(f"\tline {update['line_num']}: {update['key']}")
            print(f"\t\t current: {update['current_fallback']}")
            print(f"\t\t nt value: {update['nt_value']}")
            print(f"\t\t full nt key: {update['full_nt_key']}")
        print()
    
    if args.dry_run:
        print("dry run mode - no changes applied")
        return
    
    # Apply updates
    for file_path, updates in all_updates.items():
        print(f"\nfile: {file_path}")
        updates_to_apply = []
        
        for update in updates:
            while True:
                response = input(f"\n\t line {update['line_num']}: {update['key']}\n\t\t current: {update['current_fallback']}\n\t\t nt value: {update['nt_value']}\n\t\t full nt key: {update['full_nt_key']}\n\t\t apply this change? [y/N]: ").strip().lower()
                
                if response in ['y', 'yes']:
                    updates_to_apply.append(update)
                    break
                elif response in ['n', 'no', '']:
                    break
                else:
                    print("Please enter 'y' or 'n'")
        
        if updates_to_apply:
            success = apply_updates(file_path, updates_to_apply)
            if not success:
                print(f"\tfailed to update {file_path}")

if __name__ == "__main__":
    main()