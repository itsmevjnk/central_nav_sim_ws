import numpy as np
import pandas as pd
from glob import glob

from typing import NamedTuple
class TelemetryLine(NamedTuple):
    timestamp: float
    node: str
    content: list[str]

runs = []
for session_path in glob('*/'):
    SESSION = session_path.removesuffix('/')
    session_stamp, _, num_robots = SESSION.split('-')
    num_robots = int(num_robots.removesuffix('rbt'))
    print(f'processing session {SESSION}')
    run_robots = []
    for run_path in glob(f'{SESSION}/*/'):
        RUN = run_path.split('/')[1]
        run_stamp, central = RUN.split('-')
        central = (central == 'central')
        print(f' - processing run {RUN}: run stamp {run_stamp} with {num_robots} robots, centralised node running: {central}')

        # read telemetry log
        telemetry: list[TelemetryLine] = []
        with open(f'{run_path}/telemetry.log', 'r') as f:
            for line in f:
                nanosec, node, content = line.strip().split(':')
                telemetry.append(TelemetryLine(float(nanosec) / 1e9, node, content.split(',')))
        # print(f'telemetry has {len(telemetry)} lines')

        # determine if there is any intersection detected
        has_ix = np.nan # none for central=False
        if central:
            has_ix = False
            for line in telemetry:
                if line.node == 'central_nav' and line.content[0] == 'ix':
                    cnt = int(line.content[1])
                    if cnt > 0:
                        has_ix = True
                        break
        # print(f'has intersections: {has_ix}')

        # gather individual robots' telemetry
        robots_tmp: dict[str, dict] = {
            f'robot{i}': {
                'collided': False,
                'nav_status': 0, # latest nav2 status7
                'nav_status_stamp': -1, # latest nav2 status timestamp
                'nav_time': -1, # navigation time (except pause)
                'nav_start_stamp': -1, # navigation start timestamp
                'nav_time_total': -1, # total navigation time, from start to finish
                'nav_failed': False, # set if navigation failed
                'cmdvel_status': False, # actual robot movement status
                'cmdvel_status_stamp': -1, # and its corresponding timestamp
                'cmdvel_time': -1, # actual robot movement time (except pause)
                'moving': True, # move command from central node
                'moving_stamp': -1, # corresponding timestamp
                'pause_time': 0, # navigation pause time
                'success': False, # whether robot successfully navigated to destination (successful nav status)
                'cmd_stopped': False # set if the robot was ever stopped during navigation
            }
            for i in range(num_robots)
        }
        for (stamp, node, content) in telemetry:
            if node == 'central_nav' and content[0] == 'ix': continue # ignore intersection log
            robot = robots_tmp[content[0]]
            if node == 'pose_telemetry': # collision detection
                if robot['nav_start_stamp'] >= 0: # only care about collisions after the robot has started moving
                    robot['collided'] |= (content[1] == 'True')
            elif node == 'state_telemetry': # nav2 state
                status = int(content[-1])
                if status < 4: # active
                    if robot['nav_start_stamp'] < 0:
                        robot['nav_start_stamp'] = stamp # navigation start
                else: # not active
                    if robot['nav_status_stamp'] >= 0:
                        if robot['nav_time'] < 0: robot['nav_time'] = 0
                        robot['nav_time'] += stamp - robot['nav_status_stamp'] # one more navigation section
                        if status == 4: # success
                            robot['success'] = True
                        if status == 6: # failed
                            robot['nav_failed'] = True
                        if status == 4 or status == 6: # success or aborted (robot will no longer be moving anymore)
                            # print(f'{content[0]}: navigation completed ({robot["nav_start_stamp"]} - {stamp})')
                            robot['nav_time_total'] = stamp - robot['nav_start_stamp']
                robot['nav_status'] = status
                robot['nav_status_stamp'] = stamp
            elif node == 'cmdvel_telemetry': # cmdvel state
                moving = content[-1] == 'True'
                if not moving and robot['cmdvel_status'] and robot['cmdvel_status_stamp'] >= 0: # robot's stopped moving
                    if robot['cmdvel_time'] < 0: robot['cmdvel_time'] = 0
                    robot['cmdvel_time'] += stamp - robot['cmdvel_status_stamp']
                robot['cmdvel_status'] = moving; robot['cmdvel_status_stamp'] = stamp
            elif node == 'central_nav': # central node command
                moving = content[-1] == 'True'
                if moving and not robot['moving'] and robot['moving_stamp'] >= 0: # robot's started moving again
                    robot['pause_time'] += stamp - robot['moving_stamp']
                if not moving:
                    robot['cmd_stopped'] = True
                robot['moving'] = moving; robot['moving_stamp'] = stamp
        # for robot in robots_tmp:
        #     print(f'{robot}: {robots_tmp[robot]}')

        # add to lists
        runs.append({
            'session': SESSION,
            'run': run_stamp,
            'central': central,
            'num_robots': num_robots,
            'has_ix': has_ix,
            'successful_robots': len([r for r in robots_tmp.values() if r['success']]),
            'collided_robots': len([r for r in robots_tmp.values() if r['collided']]),
            'stopped_robots': len([r for r in robots_tmp.values() if r['cmd_stopped']]),
            'avg_nav_time': np.mean([robots_tmp[robot]['nav_time_total'] for robot in robots_tmp if robots_tmp[robot]['success']])
        })

        run_robots.extend([
            {
                'session': SESSION,
                'run': run_stamp,
                'central': central,
                'robot': robot,
                'collided': robots_tmp[robot]['collided'],
                'nav_time': robots_tmp[robot]['nav_time'],
                'nav_time_total': robots_tmp[robot]['nav_time_total'],
                'cmdvel_time': robots_tmp[robot]['cmdvel_time'],
                'pause_time': robots_tmp[robot]['pause_time'],
                'success': robots_tmp[robot]['success'],
                'cmd_stopped': robots_tmp[robot]['cmd_stopped'],
            } for robot in robots_tmp
        ])
    pd.DataFrame(run_robots).infer_objects(copy=False).replace(-1, np.nan).to_csv(f'{SESSION}_robots.csv', index=False)

pd.DataFrame(runs).to_csv('runs.csv', index=False)

