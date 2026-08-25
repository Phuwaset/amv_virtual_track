#!/usr/bin/env python3
import os
import csv
import yaml
import math

CSV_PATH = os.path.expanduser('~/turtlebot3_ws/src/custom_planner/config/station_list.csv')
YAML_PATH = os.path.expanduser('~/turtlebot3_ws/src/custom_planner/config/routes.yaml')

def csv_to_yaml():
    if not os.path.exists(CSV_PATH):
        print(f"[Error] CSV not found: {CSV_PATH}")
        return

    stations = {}
    mission_route = []

    with open(CSV_PATH, mode='r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row.get('name', '').strip()
            if not name:
                continue

            qz = float(row.get('qz', 0.0))
            qw = float(row.get('qw', 1.0))
            norm = math.hypot(qz, qw)
            if norm > 0:
                qz /= norm
                qw /= norm

            stations[name] = {
                'x': round(float(row.get('x', 0.0)), 4),
                'y': round(float(row.get('y', 0.0)), 4),
                'qz': round(qz, 4),
                'qw': round(qw, 4),
                'timer': float(row.get('timer', 3.0)),
                'pin': row.get('pin', 'Up').strip(),
                'mode': row.get('mode', 'Auto').strip()
            }
            mission_route.append(name)

    data = {
        'stations': stations,
        'mission_route': mission_route
    }

    with open(YAML_PATH, mode='w', encoding='utf-8') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

    print(f"[Sync Success] Converted CSV -> YAML ({len(stations)} stations)")

def auto_sync():
    """ตรวจสอบเวลาแก้ไขล่าสุด หากไฟล์ไหนใหม่กว่าจะทำการ Sync ให้อัตโนมัติ"""
    if not os.path.exists(CSV_PATH) and not os.path.exists(YAML_PATH):
        print("[Warning] No CSV or YAML files found to sync.")
        return

    if os.path.exists(CSV_PATH) and not os.path.exists(YAML_PATH):
        csv_to_yaml()
        return

    csv_mtime = os.path.getmtime(CSV_PATH)
    yaml_mtime = os.path.getmtime(YAML_PATH)

    if csv_mtime >= yaml_mtime:
        csv_to_yaml()
    else:
        print("[Info] YAML is up to date.")

if __name__ == '__main__':
    auto_sync()