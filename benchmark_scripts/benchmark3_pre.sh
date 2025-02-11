#!/bin/bash

source ../install/setup.bash

for i in $(seq 7 10);
do
	NUM_ROBOTS=$i NUM_RUNS=10 SIM_TIMEOUT=300 python3 ./run_multiple.py
done
