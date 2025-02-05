#!/usr/bin/env python3

from run_benchmark import run_benchmark
import os
from datetime import datetime

if __name__ == '__main__':
    NUM_ROBOTS = int(os.environ.get('NUM_ROBOTS', '2'))
    MIN_PT_DISTANCE = float(os.environ.get('MIN_PT_DISTANCE', '0.5')) # minimum distance between initial/goal points
    MIN_NAV_DISTANCE = float(os.environ.get('MIN_NAV_DISTANCE', '3.0')) # minimum distance between initial and goal points
    GZ_WORLD = os.environ.get('GZ_WORLD', 'world_8x') # empty means no Gazebo launching
    OUTPUT_DIR = os.environ.get('OUTPUT_DIR', os.getcwd() + '/' + datetime.now().strftime('%Y%m%d_%H%M%S') + f'-{GZ_WORLD}-{NUM_ROBOTS}rbt')
    NUM_RUNS = int(os.environ.get('NUM_RUNS', '100'))
    LAUNCH_TIMEOUT = int(os.environ.get('LAUNCH_TIMEOUT', '15'))

    CENTRAL = int(os.environ.get('CENTRAL', '1')) != 0
    DECENTRAL = int(os.environ.get('DECENTRAL', '1')) != 0
    subruns = [] # True = centralised, False = decentralised
    if CENTRAL: subruns.append(True)
    if DECENTRAL: subruns.append(False)
    subrun_name = {
        True: '-central',
        False: '-nocentral'
    }

    for n in range(NUM_RUNS):
        outdir = OUTPUT_DIR + '/' + datetime.now().strftime('%Y%m%d_%H%M%S')
        for i, central in enumerate(subruns):
            subrun_out = outdir + subrun_name[central]
            print(f'running {subrun_out} ({n + 1}/{NUM_RUNS})')
            poses_file = None
            if i > 0:
                poses_file = outdir + subrun_name[subruns[i - 1]] + '/poses.yml'
            run_benchmark(NUM_ROBOTS, subrun_out, GZ_WORLD, MIN_PT_DISTANCE, MIN_NAV_DISTANCE, central, poses_file, LAUNCH_TIMEOUT)
        