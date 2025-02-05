#!/usr/bin/env python3

import subprocess
import os
import csv
import random
import time
import signal
from datetime import datetime
import yaml
import numpy as np

class OutputCapturedPopen(subprocess.Popen):
    def __init__(self, cmd: list[str], stdout: str | None = None, stderr: str | None = None, env: dict[str, str] | None = None, append: bool = False, del_sigkill: bool = False):
        self.del_sigkill = del_sigkill

        self.f_stdout = None if stdout is None else open(stdout, 'a' if append else 'w')
        self.f_stderr = None if stderr is None else open(stderr, 'a' if append else 'w')

        super().__init__(cmd, stdout=self.f_stdout, stderr=self.f_stderr, env=env, preexec_fn=os.setsid)
    
    def close_files(self):
        if self.f_stderr is not None: self.f_stderr.close()
        if self.f_stdout is not None: self.f_stdout.close()

    def terminate(self):
        try:
            os.killpg(os.getpgid(self.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        self.close_files()

    def kill(self):
        try:
            os.killpg(os.getpgid(self.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.close_files()

    def __del__(self):
        if self.del_sigkill:
            self.kill() # SIGKILL
        else:
            self.terminate() # SIGTERM

        self.wait() # wait until process has been terminated
        return super().__del__()
    
    @property
    def exited(self):
        return self.poll() is not None

class SimulatedRobots:
    def __init__(self, poses: list[tuple[tuple[float, float, float], tuple[float, float, float]]], prefix: str = 'robot', start_domain: int = 10, log_dir: str = 'log', delete_entities: bool = True):
        self.processes: dict[str, list[OutputCapturedPopen]] = dict()

        this_domain = os.environ.get('ROS_DOMAIN_ID', '0')

        for i, ((init_x, init_y, init_yaw), (goal_x, goal_y, goal_yaw)) in enumerate(poses):
            domain = start_domain + i
            robot_name = f'{prefix}{i}'
            print(f'starting {robot_name} on domain {domain}')
            nav_env = os.environ.copy(); nav_env['ROS_DOMAIN_ID'] = str(domain) # for nav2
            self.processes[robot_name] = [
                OutputCapturedPopen(
                    ['ros2', 'launch', 'tb3_multi_launch', 'spawn_robot.launch.py', f'domain:={domain}', f'namespace:={robot_name}', f'x_pose:={init_x}', f'y_pose:={init_y}', f'yaw_pose:={init_yaw}'],
                    f'{log_dir}/{robot_name}_spawn.stdout.log',
                    f'{log_dir}/{robot_name}_spawn.stderr.log',
                    del_sigkill=True
                ),
                OutputCapturedPopen(
                    [
                        'ros2', 'launch', 'tb3_nav_launch', 'nav_launch.xml', 
                        f'name:={robot_name}', f'domain:={this_domain}', 'rviz:=false',
                        'init_pose:=true', f'init_x:={init_x}', f'init_y:={init_y}', f'init_yaw:={init_yaw}',
                        'goal_pose:=true', f'goal_x:={goal_x}', f'goal_y:={goal_y}', f'goal_yaw:={goal_yaw}'
                    ],
                    f'{log_dir}/{robot_name}_nav.stdout.log',
                    f'{log_dir}/{robot_name}_nav.stderr.log',
                    env=nav_env,
                    del_sigkill=True
                ),
                OutputCapturedPopen(
                    [
                        'ros2', 'launch', 'central_nav', 'robot_launch.xml',
                        f'name:={robot_name}', f'domain:={this_domain}',
                        'use_sim_time:=true'
                    ],
                    f'{log_dir}/{robot_name}_central.stdout.log',
                    f'{log_dir}/{robot_name}_central.stderr.log',
                    env=nav_env,
                    del_sigkill=True
                ),
                OutputCapturedPopen(
                    ['ros2', 'launch', 'benchmark_tools', 'nav_wait_start_launch.xml'],
                    f'{log_dir}/{robot_name}_nav_wait_start.stdout.log',
                    f'{log_dir}/{robot_name}_nav_wait_start.stderr.log',
                    env=nav_env,
                    del_sigkill=True
                ),
                OutputCapturedPopen(
                    ['ros2', 'launch', 'benchmark_tools', 'nav_wait_launch.xml'],
                    f'{log_dir}/{robot_name}_nav_wait.stdout.log',
                    f'{log_dir}/{robot_name}_nav_wait.stderr.log',
                    env=nav_env,
                    del_sigkill=True
                )
            ]
        
        self.log_dir = log_dir
        self.delete_entities = delete_entities
    
    @property
    def finished_nav(self) -> dict[str, bool]:
        return {robot_name: self.processes[robot_name][-1].exited for robot_name in self.processes}
    
    @property
    def all_finished_nav(self) -> bool:
        result = True
        for r in self.finished_nav.values(): result &= r # AND all results
        return result
    
    @property
    def started_nav(self) -> dict[str, bool]:
        return {robot_name: self.processes[robot_name][-2].exited for robot_name in self.processes}
    
    @property
    def all_started_nav(self) -> bool:
        result = True
        for r in self.started_nav.values(): result &= r # AND all results
        return result

    def __del__(self):
        if self.delete_entities:
            for robot_name in self.processes:
                print(f'deleting entity {robot_name}')
                OutputCapturedPopen(
                    ['ros2', 'service', 'call', '/delete_entity', 'gazebo_msgs/srv/DeleteEntity', f"name: '{robot_name}'"],
                    f'{self.log_dir}/{robot_name}_delete.stdout.log',
                    f'{self.log_dir}/{robot_name}_delete.stderr.log'
                ).wait() # do it sequentially to avoid jamming up Gazebo

def run_benchmark(num_robots, output_dir, gz_world, min_pt_distance, min_nav_distance, central, poses_file=None, launch_timeout=15):
    LOG_DIR = output_dir + '/log'
    os.makedirs(LOG_DIR, exist_ok=True)

    in_points: list[tuple[float, float]] = []
    POINTS_FILE = os.environ.get('POINTS_FILE', f'{os.path.dirname(__file__)}/world_points.csv')
    with open(POINTS_FILE, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            in_points.append((float(row['x']), float(row['y'])))
    
    def distance_sq(x1, y1, x2, y2):
        return (x1-x2)**2 + (y1-y2)**2

    def sample_point(points: list[tuple[float, float]] = [], min_dist: float = 0.0):
        min_dist_sq = min_dist ** 2
        shuffled_points = in_points.copy(); random.shuffle(shuffled_points)
        for point in shuffled_points:
            dist = float('inf')
            for p in points:
                d = distance_sq(point[0], point[1], p[0], p[1])
                if d < dist: dist = d
            if dist > min_dist_sq: return point
        return None
    
    def sample_points(n: int, min_dist: float = 0.0):
        points = []
        for i in range(n):
            point = sample_point(points, min_dist)
            if point is None: return None
            points.append(point)
        return points

    if len(gz_world) > 0:
        print('killing old gzserver and gzclient instances')
        OutputCapturedPopen(['killall', '-SIGKILL', 'gzserver', 'gzclient']).wait()
        print('launching Gazebo')
        gazebo = OutputCapturedPopen(
            ['ros2', 'launch', 'tb3_multi_launch', 'gazebo.launch.py', f'world:={gz_world}'],
            f'{LOG_DIR}/gazebo.stdout.log',
            f'{LOG_DIR}/gazebo.stderr.log'
        )
        # time.sleep(1) # give it some extra time
    else:
        gazebo = None

    central_nav = OutputCapturedPopen(
        [
            'ros2', 'launch', 'central_nav', ('central_launch.xml' if central else 'telemetry_launch.xml'), # only launch collision telemetry if centralised node is not launched
            'use_sim_time:=true', 'rviz:=false'
        ],
        f'{LOG_DIR}/central_nav.stdout.log',
        f'{LOG_DIR}/central_nav.stderr.log'
    )

    record_bag = OutputCapturedPopen(
        [
            'ros2', 'bag', 'record',
            '-o', f'{output_dir}/bag',
            '/robot_poses', '/robot_paths', 
            '/robot_markers', '/path_markers', '/raw_path_markers', '/ix_markers',
            '/robot_pass', '/robot_stop',
            '/clock', '/telemetry' # IMPORTANT!!!!!
        ],
        f'{LOG_DIR}/bag_record.stdout.log',
        f'{LOG_DIR}/bag_record.stderr.log'
    )

    record_telemetry = OutputCapturedPopen(
        ['ros2', 'topic', 'echo', '--csv', '--field', 'data', '--no-daemon', '/telemetry', 'std_msgs/msg/String'],
        f'{output_dir}/telemetry.log',
        f'{LOG_DIR}/telemetry.stderr.log'
    )

    if poses_file is None:
        while True:
            while True:
                init_points = sample_points(num_robots, min_pt_distance) # initial points
                if init_points is not None: break

            # sample goal points
            min_nav_distance_sq = min_nav_distance ** 2
            for i in range(10):
                goal_points = []
                for j in range(num_robots):
                    for k in range(10):
                        point = sample_point(goal_points, min_pt_distance)
                        if point is None:
                            goal_points = None
                            break
                        if distance_sq(point[0], point[1], init_points[j][0], init_points[j][1]) < min_nav_distance_sq:
                            point = None
                            continue
                        goal_points.append(point)
                        break
                    if goal_points is None or point is None:
                        goal_points = None
                        break
                if goal_points is not None: break
            if goal_points is not None: break # otherwise, we re-randomise init_points too
        
        yaws = ((np.random.random(2 * num_robots) * 2 - 1) * np.pi).tolist()
        poses = [((init_points[i][0], init_points[i][1], yaws[i*2+0]), (goal_points[i][0], goal_points[i][1], yaws[i*2+1])) for i in range(num_robots)]
    else:
        with open(poses_file, 'r') as f:
            poses = yaml.safe_load(f)

    with open(f'{output_dir}/poses.yml', 'w') as f:
        yaml.safe_dump(poses, f)

    robots = SimulatedRobots(poses, log_dir=LOG_DIR, delete_entities=(gazebo is None)) # we don't need to delete entities if Gazebo is exited

    print(f'waiting until all robots start navigating')
    t_start = time.time()
    while not robots.all_started_nav and (launch_timeout <= 0 or time.time() - t_start < launch_timeout):
        time.sleep(0.5)
    if not robots.all_started_nav:
        print(f'robots are not starting, trying again')
        del robots
        del central_nav
        del record_bag
        del record_telemetry
        if gazebo is not None: del gazebo
        return run_benchmark(num_robots, output_dir, gz_world, min_pt_distance, min_nav_distance, central, poses_file)
    print(f'robots are now navigating after {time.time() - t_start} sec')

    timer = OutputCapturedPopen(
        ['ros2', 'launch', 'benchmark_tools', 'timer_launch.xml', 'duration:=120.0'],
        f'{LOG_DIR}/timer.stdout.log',
        f'{LOG_DIR}/timer.stderr.log'
    )

    try:
        while True:
            print(f'robots finished: {robots.finished_nav} (all: {robots.all_finished_nav}), timer finished: {timer.exited}')
            if robots.all_finished_nav or timer.exited: break
            time.sleep(0.5)
    finally:
        print('cleaning up')
        del robots
        del central_nav
        del record_bag
        del timer
        del record_telemetry
        if gazebo is not None: del gazebo

if __name__ == '__main__':
    NUM_ROBOTS = int(os.environ.get('NUM_ROBOTS', '2'))
    MIN_PT_DISTANCE = float(os.environ.get('MIN_PT_DISTANCE', '1.0')) # minimum distance between initial/goal points
    MIN_NAV_DISTANCE = float(os.environ.get('MIN_NAV_DISTANCE', '2.0')) # minimum distance between initial and goal points
    GZ_WORLD = os.environ.get('GZ_WORLD', '') # empty means no Gazebo launching
    CENTRAL = int(os.environ.get('CENTRAL', '1')) != 0
    POSES = os.environ.get('POSES', None)
    OUTPUT_DIR = os.environ.get('OUTPUT_DIR', os.getcwd() + '/' + datetime.now().strftime('%Y%m%d_%H%M%S') + f'-{NUM_ROBOTS}' + ('-nocentral' if not CENTRAL else ''))
    LAUNCH_TIMEOUT = int(os.environ.get('LAUNCH_TIMEOUT', '15'))

    run_benchmark(NUM_ROBOTS, OUTPUT_DIR, GZ_WORLD, MIN_PT_DISTANCE, MIN_NAV_DISTANCE, CENTRAL, POSES, LAUNCH_TIMEOUT)