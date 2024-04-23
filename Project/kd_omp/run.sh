#!/bin/bash

echo "================= ======================== ================="
echo "============== Executing KD Case with Open MP =============="
echo "================= ======================== ================="


min_points=10

for i in 1 10 100 1000 10000 100000
do
    points=$((min_points * i))
    echo "================= ======================== ================="
    echo "CHAMFER Distance for $points Points"
    time ./CD_kd_omp $points
    echo "================= ======================== ================="
done
