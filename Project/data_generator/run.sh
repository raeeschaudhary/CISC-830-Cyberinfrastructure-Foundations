#!/bin/bash

echo "================= ======================== ================="
echo "=================  Generating Data Points  ================="
echo "================= ======================== ================="

min_points=10

for i in 1 10 100 1000 10000 100000
do
    points=$((min_points * i))
    echo "Generating data for $points points"

    ./data_generator $points
done
