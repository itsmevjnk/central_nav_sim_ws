#!/bin/bash

source ../install/setup.bash

for i in $(seq 2 6);
do
	NUM_RUNS=100 NUM_ROBOTS=$i python3 ./run_multiple.py
done
