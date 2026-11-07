#!/usr/bin/env python3
"""Только для разработки: шлёт NavigateToPose сразу нескольким роботам и сверяет итог с GT.

  goals.py ground_1:6,-2 ground_2:6,0 ground_3:6,2 [--timeout 120]
Печатает для каждого робота: результат, время, ошибку конечной позы относительно цели
(поза берётся из /<robot>/odom: при gt_pose:=true это истинная поза в кадре world).
Код возврата 0, если все успешны.
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('goals', nargs='+', help='robot:x,y')
    ap.add_argument('--timeout', type=float, default=120.0)
    a, _ = ap.parse_known_args()
    rclpy.init()
    node = Node('goals_client', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    poses, clients, targets = {}, {}, {}
    for g in a.goals:
        r, xy = g.split(':')
        targets[r] = tuple(float(v) for v in xy.split(','))
        clients[r] = ActionClient(node, NavigateToPose, f'/{r}/navigate_to_pose')
        node.create_subscription(Odometry, f'/{r}/odom',
                                 lambda m, r=r: poses.__setitem__(r, (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
    for r, c in clients.items():
        if not c.wait_for_server(timeout_sec=60.0):
            print(f'{r}: нет action-сервера'); return 2
    t0 = {}
    futs = {}
    for r, (x, y) in targets.items():
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'world'
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.w = 1.0
        t0[r] = time.monotonic()
        futs[r] = clients[r].send_goal_async(goal)
    handles, results, done_t = {}, {}, {}
    min_d = float('inf')
    end = time.monotonic() + a.timeout
    while time.monotonic() < end and len(done_t) < len(targets):
        rclpy.spin_once(node, timeout_sec=0.05)
        ps = list(poses.values())
        for i in range(len(ps)):
            for j in range(i + 1, len(ps)):
                min_d = min(min_d, math.hypot(ps[i][0] - ps[j][0], ps[i][1] - ps[j][1]))
        for r, f in futs.items():
            if r not in handles and f.done():
                h = f.result()
                if not h.accepted:
                    done_t[r] = time.monotonic(); results[r] = -1; continue
                handles[r] = h
                results[r] = h.get_result_async()
        for r, rf in list(results.items()):
            if r not in done_t and not isinstance(rf, int) and rf.done():
                done_t[r] = time.monotonic()
    ok = True
    print(f'мин. расстояние между роботами за время теста: {min_d:.2f} м (корпус ~0.58 м диагональ)')
    for r, (x, y) in targets.items():
        rf = results.get(r)
        status = (rf.result().status if rf is not None and not isinstance(rf, int) and rf.done() else None)
        success = status == GoalStatus.STATUS_SUCCEEDED
        ok &= success
        p = poses.get(r)
        err = math.hypot(p[0] - x, p[1] - y) if p else float('nan')
        dt = done_t[r] - t0[r] if r in done_t else float('nan')
        print(f'{r}: {"OK" if success else "FAIL"} (status={status}) время={dt:.1f}с (реальное) '
              f'ошибка_до_цели={err:.2f}м финал={p}')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
