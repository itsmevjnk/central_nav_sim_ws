#!/bin/bash

source /opt/ros/humble/setup.bash
source /cnav_ws/install/local_setup.bash

cleanup() {
    echo "Fully cleaning up simulation."
    ros2 service call /clean_simulation std_srvs/srv/Empty

    if [[ -z "${EXIT_BENCHMARK}" ]]; then
        echo "Benchmark completed, keeping terminal open to allow results to be copied."
        tail -f /dev/null
    fi
}
trap cleanup EXIT

time python3 -u /scripts/run_multiple.py # NOTE: we cannot run tee from here, not without causing broken pipe errors

