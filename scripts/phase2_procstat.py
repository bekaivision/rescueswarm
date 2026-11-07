#!/usr/bin/env python3
"""Фаза 2: загрузка CPU/RAM по группам процессов внутри контейнера (чтение /proc, без ROS).

  phase2_procstat.py OUT_FILE DURATION_S [INTERVAL_S=5]
Каждые INTERVAL_S пишет строку JSON {t, total_cores, rss_mib, groups:{имя: ядра}} в OUT_FILE
(дополняет файл). Ядра = доля одного ядра за интервал (1.0 = 100 %).
"""
import json
import os
import sys
import time

TCK = os.sysconf('SC_CLK_TCK')
GROUPS = [
    ('gz', ('gz sim', 'gz-sim', 'ruby')),
    ('bridges', ('parameter_bridge', 'ros_gz_bridge')),
    ('nav2', ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
              'lifecycle_manager', 'nav2_')),
    ('shared_map', ('shared_map',)),
    ('explorers', ('explorer',)),
    ('coverage', ('rs_eval', 'coverage')),
    ('rviz', ('rviz2',)),
    ('rsp', ('robot_state_publisher', 'static_transform')),
]


def group_of(cmd):
    for name, keys in GROUPS:
        if any(k in cmd for k in keys):
            return name
    return 'other'


def snapshot():
    res = {}
    rss = 0
    page = os.sysconf('SC_PAGE_SIZE')
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            cmd = open(f'/proc/{pid}/cmdline').read().replace('\0', ' ')
            st = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()
            ticks = int(st[11]) + int(st[12])
            rss += int(st[21]) * page
            if pid == str(os.getpid()):
                continue
            res[pid] = (group_of(cmd), ticks)
        except (OSError, IndexError):
            pass
    return res, rss


def main():
    out, dur = sys.argv[1], float(sys.argv[2])
    iv = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
    prev, _ = snapshot()
    t_prev = time.time()
    t0 = t_prev
    while time.time() - t0 < dur:
        time.sleep(iv)
        cur, rss = snapshot()
        now = time.time()
        g = {}
        for pid, (name, ticks) in cur.items():
            if pid in prev:
                g[name] = g.get(name, 0.0) + (ticks - prev[pid][1]) / TCK / (now - t_prev)
        with open(out, 'a') as f:
            f.write(json.dumps({'t': round(now - t0, 1), 'total_cores': round(sum(g.values()), 3),
                                'rss_mib': round(rss / 2**20), 'groups': {k: round(v, 3) for k, v in g.items()}}) + '\n')
        prev, t_prev = cur, now


if __name__ == '__main__':
    main()
