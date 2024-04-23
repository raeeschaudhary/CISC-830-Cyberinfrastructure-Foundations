#!/bin/bash

echo "================= ======================== ================="
echo "============ Executing Baseline Case with OpenMP ==========="
echo "================= ======================== ================="

min_points=10

for i in 1 10 100 1000 10000 100000
do
    points=$((min_points * i))
    echo "================= ======================== ================="
    echo "CHAMFER Distance for $points Points"
    time ./CD_baseline_omp $points
    echo "================= ======================== ================="
done
