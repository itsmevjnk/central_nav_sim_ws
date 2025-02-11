#!/bin/bash

source ../install/setup.bash

for i in $(seq 5 6);
do
	NUM_ROBOTS=$i python3 ./run_multiple.py
done
