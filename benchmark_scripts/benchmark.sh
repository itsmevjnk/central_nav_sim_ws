#!/bin/bash

source ../install/setup.bash

for i in $(seq 2 4);
do
	NUM_ROBOTS=$i python3 ./run_multiple.py
done
