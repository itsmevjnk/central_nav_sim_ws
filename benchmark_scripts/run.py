import subprocess
import os
import csv
import random
import time
import signal
from datetime import datetime
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
                        f'name:={robot_name}', f'domain:={this_domain}', 
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
        return {robot_name: (self.processes[robot_name][-1].poll() is not None) for robot_name in self.processes}
    
    @property
    def all_finished_nav(self) -> bool:
        result = True
        for r in self.finished_nav.values(): result &= r # AND all results
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

if __name__ == '__main__':
    NUM_ROBOTS = int(os.environ.get('NUM_ROBOTS', '2'))
    
    OUTPUT_DIR = os.environ.get('OUTPUT_DIR', os.getcwd() + '/' + datetime.now().strftime('%Y%m%d_%H%M%S') + f'-{NUM_ROBOTS}')
    
    LOG_DIR = OUTPUT_DIR + '/log'
    os.makedirs(LOG_DIR, exist_ok=True)

    POINTS_FILE = os.environ.get('POINTS_FILE', f'{os.path.dirname(__file__)}/world_points.csv')
    in_points: list[tuple[float, float]] = []
    with open(POINTS_FILE, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            in_points.append((float(row['x']), float(row['y'])))
    
    def distance_sq(x1, y1, x2, y2):
        return (x1-x2)**2 + (y1-y2)**2
    
    MIN_PT_DISTANCE = float(os.environ.get('MIN_PT_DISTANCE', '1.0')) # minimum distance between initial/goal points
    MIN_NAV_DISTANCE = float(os.environ.get('MIN_NAV_DISTANCE', '2.0')) # minimum distance between initial and goal points

    def sample_point(points: list[tuple[float, float]] = [], min_dist: float = 0.0):
        min_dist_sq = min_dist ** 2
        while True:
            point = random.choice(in_points)
            dist = float('inf')
            for p in points:
                d = distance_sq(point[0], point[1], p[0], p[1])
                if d < dist: dist = d
            if dist > min_dist_sq: return point
    
    def sample_points(n: int, min_dist: float = 0.0):
        points = []
        for i in range(n):
            points.append(sample_point(points, min_dist))
        return points

    GZ_WORLD = os.environ.get('GZ_WORLD', '') # empty means no Gazebo launching
    if len(GZ_WORLD) > 0:
        print('killing old gzserver and gzclient instances')
        OutputCapturedPopen(['killall', '-SIGKILL', 'gzserver', 'gzclient']).wait()
        print('launching Gazebo')
        gazebo = OutputCapturedPopen(
            ['ros2', 'launch', 'tb3_multi_launch', 'gazebo.launch.py', f'world:={GZ_WORLD}'],
            f'{LOG_DIR}/gazebo.stdout.log',
            f'{LOG_DIR}/gazebo.stderr.log'
        )
        # time.sleep(1) # give it some extra time
    else:
        gazebo = None

    central_nav = OutputCapturedPopen(
        [
            'ros2', 'launch', 'central_nav', 'central_launch.xml',
            f'map:=src/tb3_nav_launch/map/world.yaml', # for visualisation only, can omit
            'use_sim_time:=true'
        ],
        f'{LOG_DIR}/central_nav.stdout.log',
        f'{LOG_DIR}/central_nav.stderr.log'
    )

    record_bag = OutputCapturedPopen(
        [
            'ros2', 'bag', 'record',
            '-o', f'{OUTPUT_DIR}/bag',
            '/robot_poses', '/robot_paths', 
            '/robot_markers', '/path_markers', '/raw_path_markers', '/ix_markers',
            '/robot_pass', '/robot_stop',
            '/clock', '/telemetry' # IMPORTANT!!!!!
        ],
        f'{LOG_DIR}/bag_record.stdout.log',
        f'{LOG_DIR}/bag_record.stderr.log'
    )

    record_telemetry = OutputCapturedPopen(
        ['ros2', 'topic', 'echo', '/telemetry', 'std_msgs/msg/String', '--csv', '--field data'],
        f'{LOG_DIR}/telemetry.stdout.log',
        f'{LOG_DIR}/telemetry.stderr.log'
    )

    init_points = sample_points(NUM_ROBOTS, MIN_PT_DISTANCE) # initial points

    # sample goal points
    goal_points = []
    MIN_NAV_DISTANCE_SQ = MIN_NAV_DISTANCE ** 2
    for i in range(NUM_ROBOTS):
        while True:
            point = sample_point(goal_points, MIN_PT_DISTANCE)
            if distance_sq(point[0], point[1], init_points[i][0], init_points[i][1]) < MIN_NAV_DISTANCE_SQ: continue
            goal_points.append(point)
            break
    
    yaws = ((np.random.random(2 * NUM_ROBOTS) * 2 - 1) * np.pi).tolist()

    poses = [((init_points[i][0], init_points[i][1], yaws[i*2+0]), (goal_points[i][0], goal_points[i][1], yaws[i*2+1])) for i in range(NUM_ROBOTS)]
    robots = SimulatedRobots(poses, log_dir=LOG_DIR, delete_entities=(gazebo is None)) # we don't need to delete entities if Gazebo is exited

    timer = OutputCapturedPopen(
        ['ros2', 'launch', 'benchmark_tools', 'timer_launch.xml', 'duration:=120.0'],
        f'{LOG_DIR}/timer.stdout.log',
        f'{LOG_DIR}/timer.stderr.log'
    )

    try:
        while (not robots.all_finished_nav) and (timer.poll() is None):
            print(f'robots finished: {robots.finished_nav} (all: {robots.all_finished_nav}), timer finished: {timer.poll() is not None}')
            time.sleep(1)
    finally:
        print('cleaning up')
        del robots
        del central_nav
        del record_bag
        del timer
        del record_telemetry
        if gazebo is not None: del gazebo