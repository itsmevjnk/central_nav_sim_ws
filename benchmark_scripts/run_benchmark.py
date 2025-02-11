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
import shutil
import re
from scipy.spatial.transform import Rotation

class OutputCapturedPopen(subprocess.Popen):
    def __init__(self, cmd: list[str], stdout: str | None = None, stderr: str | None = None, env: dict[str, str] | None = None, append: bool = False, del_sigkill: bool = False):
        self.del_sigkill = del_sigkill

        self.f_stdout = subprocess.DEVNULL if stdout is None else open(stdout, 'a' if append else 'w')
        self.f_stderr = subprocess.DEVNULL if stderr is None else open(stderr, 'a' if append else 'w')

        super().__init__(cmd, stdout=self.f_stdout, stderr=self.f_stderr, env=env, preexec_fn=os.setsid)
    
    def close_files(self):
        if self.f_stderr is not None and type(self.f_stderr) is not int: self.f_stderr.close()
        if self.f_stdout is not None and type(self.f_stderr) is not int: self.f_stdout.close()

    def terminate(self):
        print(f'sending SIGTERM to PID {self.pid}')
        try:
            os.killpg(os.getpgid(self.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        self.close_files()

    def kill(self):
        print(f'sending SIGKILL to PID {self.pid}')
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

        self.wait(5.0) # wait until process has been terminated
        return super().__del__()
    
    @property
    def exited(self):
        return self.poll() is not None

class SimulatedRobotPool:
    def __init__(self, poses: list[tuple[float, float, float]] = [], prefix: str = 'robot', start_domain: int = 10, log_dir: str = 'log', gz_world: str | None = None, gz_headless: bool = True, rviz: bool = False):
        self.processes: list[list[OutputCapturedPopen] | None] = []
        self.start_domain = start_domain
        self.prefix = prefix
        self.log_dir = log_dir
        self.rviz = rviz
        self.this_domain = os.environ.get('ROS_DOMAIN_ID', '0')

        os.makedirs(log_dir, exist_ok=True)

        if gz_world is not None:
            print(f'killing all other gzserver/gzclient instances')
            try:
                subprocess.check_call(['killall', '-SIGKILL', 'gzserver', 'gzclient'])
            except subprocess.CalledProcessError:
                pass

            self.gz_process = OutputCapturedPopen(
                ['ros2', 'launch', 'tb3_multi_launch', 'gazebo.launch.py', f'world:={gz_world}', f'headless:={gz_headless}'],
                f'{log_dir}/gazebo.stdout.log',
                f'{log_dir}/gazebo.stderr.log'
            )
        else:
            self.gz_process = None

        for pose in poses:
            self.add_robot(pose)

    def has_index(self, idx: int) -> bool:
        return idx < len(self.processes) and self.processes[idx] is not None
    
    def find_unused_idx(self) -> int:
        for i, proc in self.processes:
            if proc is None: return i
        i = len(self.processes)
        self.processes.append(None)
        return i

    def is_ready(self, idx: int) -> bool:
        if not self.has_index(idx): return False
        return self.processes[idx][-1].poll() is not None # ready when AMCL init exits

    def wait_ready(self, idx: int):
        while not self.is_ready(idx):
            time.sleep(0.25)

    def add_robot(self, pose: tuple[float, float, float] = (0.0, 0.0, 0.0), idx: int | None = None, wait_ready: bool = False) -> tuple[int, str, int]:
        if idx is None:
            idx = self.find_unused_idx()
        elif idx >= len(self.processes): # expand
            self.processes.extend([None for i in range(idx - len(self.processes) + 1)])
        elif self.processes[idx] is not None: # already exists - move robot
            self.move_robot(idx, pose)
            if wait_ready:
                self.wait_ready(idx)
            return (idx, f'{self.prefix}{idx}', self.start_domain + idx)

        name = f'{self.prefix}{idx}'
        domain = self.start_domain + idx
        init_x, init_y, init_yaw = pose

        nav_env = os.environ.copy(); nav_env['ROS_DOMAIN_ID'] = str(domain) # for nav2

        print(f'adding robot {name} on domain {domain}')
        self.processes[idx] = [
            OutputCapturedPopen(
                ['ros2', 'launch', 'tb3_multi_launch', 'spawn_robot.launch.py', 'model:=waffle_bm', f'domain:={domain}', f'namespace:={name}', f'x_pose:={init_x}', f'y_pose:={init_y}', f'yaw_pose:={init_yaw}'],
                f'{self.log_dir}/{name}_spawn.stdout.log',
                f'{self.log_dir}/{name}_spawn.stderr.log',
                del_sigkill=True
            ),
        ]

        while True:
            time.sleep(0.25)
            try:
                output = subprocess.check_output(['ros2', 'topic', 'info', f'/scan'], env=nav_env, stderr=subprocess.DEVNULL).decode()
            except subprocess.CalledProcessError: # topic's not up yet
                continue
            num_publishers = int(output.splitlines()[1].split(': ')[1]) 
            if num_publishers == 0: # publisher count
                continue
            break
        print(f' - robot is now on domain {domain}, starting Nav2')

        self.processes[idx].extend([
            OutputCapturedPopen(
                [
                    'ros2', 'launch', 'tb3_nav_launch', 'nav_launch.xml', 
                    f'name:={name}', f'domain:={self.this_domain}', f'rviz:={self.rviz}', 'use_sim_time:=true',
                    'init_pose:=false'
                ],
                f'{self.log_dir}/{name}_nav.stdout.log',
                f'{self.log_dir}/{name}_nav.stderr.log',
                env=nav_env,
                del_sigkill=True
            ),
            OutputCapturedPopen(
                [
                    'ros2', 'launch', 'central_nav', 'robot_launch.xml',
                    f'name:={name}', f'domain:={self.this_domain}',
                    'use_sim_time:=true',
                    'init_goal:=false'
                ],
                f'{self.log_dir}/{name}_central.stdout.log',
                f'{self.log_dir}/{name}_central.stderr.log',
                env=nav_env,
                del_sigkill=True
            ),
            OutputCapturedPopen(
                ['ros2', 'launch', 'benchmark_tools', 'bumper_launch.xml', f'name:={name}'],
                f'{self.log_dir}/{name}_bumper.stdout.log',
                f'{self.log_dir}/{name}_bumper.stderr.log',
                env=nav_env,
                del_sigkill=True
            ),
            OutputCapturedPopen(
                [
                    'ros2', 'launch', 'nav2_oneshot_nodes', 'amcl_init_launch.xml', 'use_sim_time:=true',
                    f'x:={init_x}', f'y:={init_y}', f'yaw:={init_yaw}'
                ],
                f'{self.log_dir}/{name}_amcl_init.stdout.log',
                f'{self.log_dir}/{name}_amcl_init.stderr.log',
                env=nav_env,
                del_sigkill=True
            )
        ])   

        if wait_ready:
            self.wait_ready(idx)

        return (idx, name, domain)
    
    def remove_robot(self, idx: int):
        if not self.has_index(idx): return
        name = f'{self.prefix}{idx}'
        print(f'deleting robot {name}')
        subprocess.check_call(['ros2', 'service', 'call', '/delete_entity', 'gazebo_msgs/srv/DeleteEntity', f'name: {name}'], stdout=subprocess.DEVNULL)
        self.processes[idx] = None # will trigger SIGKILL
    
    def move_robot(self, idx: int, pose: tuple[float, float, float]):
        if not self.has_index(idx): return

        name = f'{self.prefix}{idx}'
        x, y, yaw = pose
        qx, qy, qz, qw = Rotation.from_euler('z', yaw).as_quat()
        subprocess.check_call([
            'ros2', 'service', 'call', '/gazebo/set_entity_state', 'gazebo_msgs/srv/SetEntityState',
            f'state: {{name: \'{name}\', pose: {{position: {{x: {x}, y: {y}, z: 0.01}}, orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}, reference_frame: world}}'
        ], stdout=subprocess.DEVNULL)
        nav_env = os.environ.copy(); nav_env['ROS_DOMAIN_ID'] = str(self.start_domain + idx)
        self.processes[idx][-1] = OutputCapturedPopen(
            [
                'ros2', 'launch', 'nav2_oneshot_nodes', 'amcl_init_launch.xml', 'use_sim_time:=true',
                f'x:={x}', f'y:={y}', f'yaw:={yaw}'
            ],
            env=nav_env,
            del_sigkill=True
        )

    @property
    def all_ready(self) -> bool:
        result = True
        for idx, proc in enumerate(self.processes):
            if proc is None: continue
            result &= self.is_ready(idx)
        return result
    
    def wait_all_ready(self):
        while not self.all_ready:
            time.sleep(0.25)
    
    def __del__(self):
        if self.gz_process is not None:
            del self.gz_process
            for proc in self.processes: del proc
        else:
            robots = len(self.processes)
            for i in range(robots):
                self.remove_robot(i)
    
    def get_pose(self, idx: int) -> tuple[float, float, float] | None:
        if not self.has_index(idx): return None

        # get initial pose from Gazebo
        robot_name = f'{self.prefix}{idx}'
        resp = subprocess.check_output(['ros2', 'service', 'call', '/gazebo/get_entity_state', 'gazebo_msgs/srv/GetEntityState', f'{{name: \'{robot_name}\', reference_frame: \'world\'}}']).decode().splitlines()[-2]
        match = re.search(r'geometry_msgs\.msg\.Point\(x=([\-0-9.e]*), y=([\-0-9.e]*), z=([\-0-9.e]*)\)', resp)
        init_x = float(match.group(1)); init_y = float(match.group(2))
        match = re.search(r'geometry_msgs\.msg\.Quaternion\(x=([\-0-9.e]*), y=([\-0-9.e]*), z=([\-0-9.e]*), w=([\-0-9.e]*)\)', resp)
        _, _, init_yaw = Rotation.from_quat([float(match.group(i)) for i in range(1, 5)]).as_euler('xyz')

        return (init_x, init_y, init_yaw)

    def navigate(self, idx: int, goal: tuple[float, float, float], f_stdout=None, f_stderr=None):
        if not self.has_index(idx): return

        # init_x, init_y, init_yaw = self.get_pose(idx)

        domain = self.start_domain + idx
        nav_env = os.environ.copy(); nav_env['ROS_DOMAIN_ID'] = str(domain)
        qx, qy, qz, qw = Rotation.from_euler('y', goal[2]).as_quat()
        subprocess.check_call(
            [
                'ros2', 'topic', 'pub', '--once', '/goal_pose_alt', 'geometry_msgs/msg/PoseStamped',
                f'{{header: {{frame_id: map}}, pose: {{position: {{x: {goal[0]}, y: {goal[1]}}}, orientation: {{x: {qx}, x: {qy}, x: {qz}, x: {qw}}}}}}}'
            ],
            env=nav_env,
            stdout=subprocess.DEVNULL
        )

        # command nav2
        subprocess.check_call(
            [
                'ros2', 'launch', 'nav2_oneshot_nodes', 'goal_launch.xml', 'use_sim_time:=true',
                f'x:={goal[0]}', f'y:={goal[1]}', f'yaw:={goal[2]}'
            ],
            env=nav_env,
            stdout=f_stdout,
            stderr=f_stderr
        )
        # once this returns, we can assume that the robot is navigating (since it's rare that the goal would be rejected)
    
    def navigate_async(self, idx: int, goal: tuple[float, float, float], f_stdout=None, f_stderr=None) -> subprocess.Popen | None:
        if not self.has_index(idx): return None

        # init_x, init_y, init_yaw = self.get_pose(idx)

        domain = self.start_domain + idx
        nav_env = os.environ.copy(); nav_env['ROS_DOMAIN_ID'] = str(domain)
        qx, qy, qz, qw = Rotation.from_euler('y', goal[2]).as_quat()
        subprocess.check_call(
            [
                'ros2', 'topic', 'pub', '--once', '/goal_pose_alt', 'geometry_msgs/msg/PoseStamped',
                f'{{header: {{frame_id: map}}, pose: {{position: {{x: {goal[0]}, y: {goal[1]}}}, orientation: {{x: {qx}, x: {qy}, x: {qz}, x: {qw}}}}}}}'
            ],
            env=nav_env,
            stdout=subprocess.DEVNULL
        )

        # command nav2
        return subprocess.Popen(
            [
                'ros2', 'launch', 'nav2_oneshot_nodes', 'goal_launch.xml', 'use_sim_time:=true',
                f'x:={goal[0]}', f'y:={goal[1]}', f'yaw:={goal[2]}'
            ],
            env=nav_env,
            stdout=f_stdout,
            stderr=f_stderr
        )
    
    def cancel_navigation(self, idx: int, f_stdout=None, f_stderr=None):
        if not self.has_index(idx): return

        domain = self.start_domain + idx
        nav_env = os.environ.copy(); nav_env['ROS_DOMAIN_ID'] = str(domain)

        subprocess.check_call(
            ['ros2', 'service', 'call', '/navigate_to_pose/_action/cancel_goal', 'action_msgs/srv/CancelGoal'],
            env=nav_env,
            stdout=f_stdout,
            stderr=f_stderr
        )
    
    def cancel_navigation_async(self, idx: int, f_stdout=None, f_stderr=None):
        if not self.has_index(idx): return

        domain = self.start_domain + idx
        nav_env = os.environ.copy(); nav_env['ROS_DOMAIN_ID'] = str(domain)

        return subprocess.Popen(
            ['ros2', 'service', 'call', '/navigate_to_pose/_action/cancel_goal', 'action_msgs/srv/CancelGoal'],
            env=nav_env,
            stdout=f_stdout,
            stderr=f_stderr
        )


def run_benchmark(robots: SimulatedRobotPool, num_robots: int, output_dir: str, min_pt_distance: float, min_nav_distance: float, central: bool, poses_file: str | None = None, pause_phys: bool = False, launch_timeout: float = 15, sim_timeout: float = 120):
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

    if central:
        central_nav = OutputCapturedPopen(
            [
                'ros2', 'launch', 'central_nav', 'central_launch.xml',
                'use_sim_time:=true', 'rviz:=false'
            ],
            f'{LOG_DIR}/central_nav.stdout.log',
            f'{LOG_DIR}/central_nav.stderr.log'
        )
    else:
        central_nav = None

    bag_dir = f'{output_dir}/bag'
    record_bag = OutputCapturedPopen(
        [
            'ros2', 'bag', 'record',
            '-o', bag_dir,
            '/robot_poses', '/robot_paths', 
            '/robot_markers', '/path_markers', '/raw_path_markers', '/ix_markers',
            '/robot_pass', '/robot_stop',
            '/performance_metrics', # Gazebo performance
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

    init_poses, goal_poses = list(zip(*poses))
    if pause_phys: subprocess.check_call(['ros2', 'service', 'call', '/pause_physics', 'std_srvs/srv/Empty'], stdout=subprocess.DEVNULL)
    for i, pose in enumerate(init_poses):
        robots.add_robot(pose, i)
    if pause_phys: subprocess.check_call(['ros2', 'service', 'call', '/unpause_physics', 'std_srvs/srv/Empty'], stdout=subprocess.DEVNULL)
    robots.wait_all_ready()

    navigate_procs = [
        robots.navigate_async(
            i, pose,
            f_stdout=open(f'{LOG_DIR}/robot{i}_initgoal.stdout.log', 'w'),
            f_stderr=open(f'{LOG_DIR}/robot{i}_initgoal.stderr.log', 'w')
        )
        for i, pose in enumerate(goal_poses)
    ]

    print(f'waiting until all robots start navigating')
    t_start = time.time()
    nav_ok = False
    while not nav_ok or (launch_timeout <= 0 or time.time() - t_start < launch_timeout):
        nav_ok = True
        for proc in navigate_procs:
            nav_ok &= proc.poll() is not None
        if nav_ok: break
        time.sleep(0.5)
    if not nav_ok:
        print(f'robots are not starting, trying again')
        del record_bag
        del record_telemetry
        if central_nav is not None: del central_nav
        shutil.rmtree(bag_dir)
        return run_benchmark(robots, num_robots, output_dir, min_pt_distance, min_nav_distance, central, poses_file, launch_timeout)
    print(f'robots are now navigating after {time.time() - t_start} sec')

    navigate_procs = []
    for i in range(len(robots.processes)):
        nav_env = os.environ.copy(); nav_env['ROS_DOMAIN_ID'] = str(robots.start_domain + i)
        navigate_procs.append(OutputCapturedPopen(
            ['ros2', 'launch', 'benchmark_tools', 'nav_wait_launch.xml'],
            f'{LOG_DIR}/robot{i}_nav_wait.stdout.log',
            f'{LOG_DIR}/robot{i}_nav_wait.stderr.log',
            env=nav_env,
            del_sigkill=True
        ))

    timer = OutputCapturedPopen(
        ['ros2', 'launch', 'benchmark_tools', 'timer_launch.xml', f'duration:={sim_timeout:.1f}', 'use_sim_time:=true'],
        f'{LOG_DIR}/timer.stdout.log',
        f'{LOG_DIR}/timer.stderr.log'
    )

    try:
        while True:
            procs_finished = [proc.exited for proc in navigate_procs]
            procs_all_finished = True
            for proc in procs_finished: procs_all_finished &= proc
            print(f'{time.time()}: {len([p for p in procs_finished if p])}/{num_robots} robots finished, timer finished: {timer.exited}')
            if procs_all_finished or timer.exited: break
            time.sleep(0.5)
    finally:
        print('cancelling navigation goals')
        navigate_procs = [
            robots.cancel_navigation_async(
                i,
                f_stdout=open(f'{LOG_DIR}/robot{i}_nav_cancel.stdout.log', 'w'),
                f_stderr=open(f'{LOG_DIR}/robot{i}_nav_cancel.stderr.log', 'w')
            )
            for i in range(len(robots.processes))
        ]
        while True:
            ok = True
            for proc in navigate_procs: ok &= proc.poll() is not None
            if ok: break
            time.sleep(0.5)

        print('cleaning up')
        if central_nav is not None: del central_nav
        del record_bag
        del timer
        del record_telemetry

if __name__ == '__main__':
    NUM_ROBOTS = int(os.environ.get('NUM_ROBOTS', '2'))
    MIN_PT_DISTANCE = float(os.environ.get('MIN_PT_DISTANCE', '0.5')) # minimum distance between initial/goal points
    MIN_NAV_DISTANCE = float(os.environ.get('MIN_NAV_DISTANCE', '3.0')) # minimum distance between initial and goal points
    GZ_WORLD = os.environ.get('GZ_WORLD', '') # empty means no Gazebo launching
    CENTRAL = int(os.environ.get('CENTRAL', '1')) != 0
    POSES = os.environ.get('POSES', None)
    OUTPUT_DIR = os.environ.get('OUTPUT_DIR', os.getcwd() + '/' + datetime.now().strftime('%Y%m%d_%H%M%S') + f'-{NUM_ROBOTS}' + ('-nocentral' if not CENTRAL else ''))
    LAUNCH_TIMEOUT = int(os.environ.get('LAUNCH_TIMEOUT', '60'))
    SIM_TIMEOUT = int(os.environ.get('SIM_TIMEOUT', str(120 + (NUM_ROBOTS - 2) * 30)))
    GZ_HEADLESS = int(os.environ.get('GZ_HEADLESS', '1')) != 0
    RVIZ = int(os.environ.get('RVIZ', '0')) != 0

    robots = SimulatedRobotPool(
        gz_world=None if len(GZ_WORLD) == 0 else GZ_WORLD,
        gz_headless=GZ_HEADLESS,
        log_dir=f'{OUTPUT_DIR}/log',
        rviz=RVIZ
    )
    run_benchmark(robots, NUM_ROBOTS, OUTPUT_DIR, MIN_PT_DISTANCE, MIN_NAV_DISTANCE, CENTRAL, POSES, False, LAUNCH_TIMEOUT, SIM_TIMEOUT)

    del robots
