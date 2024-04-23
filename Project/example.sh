#!/bin/bash

## Compile Data Generator 

echo "================= ======================== ================="
echo "================= Compiling Data Generator ================="
echo "================= ======================== ================="
cd data_generator
cmake . 
make 
cd ..

## Compile and Execute Each Case
## Compile Baseline 

echo "================= ======================== ================="
echo "================= Compiling Baseline Case  ================="
echo "================= ======================== ================="
cd baseline 
cmake .
make 
cd ..

## Compile Baseline Case with Open MP

echo "================= ======================== ================="
echo "=========== Compiling Baseline Case with OpenMP  ==========="
echo "================= ======================== ================="
cd baseline_omp
cmake .
make 
cd ..

## Compile Baseline Case with GPU

echo "================= ======================== ================="
echo "============  Compiling Baseline Case with GPU  ============"
echo "================= ======================== ================="
cd baseline_GPU
cmake .
make 
cd ..

## Compile KD Case

echo "================= ======================== ================="
echo "==================== Compiling KD Case ====================="
echo "================= ======================== ================="
cd kd
cmake .
make 
cd ..


## Compile KD Case with OpenMP

echo "================= ======================== ================="
echo "============== Compiling KD Case with OpenMP ==============="
echo "================= ======================== ================="
cd kd_omp
cmake .
make 
cd ..


## Execute Data Generator to Generate Data Points

min_points=10
echo "=================  Generating Data Points  ================="
cd data_generator
chmod +x data_generator
for i in 1 10 100 1000 10000
do
    points=$((min_points * i))
    echo "Generating data for $points points"
    ./data_generator $points
done
cd ..
## Make sure executeables exisit in the example_exec folder

execute_cd() {
    directory="$1"
    executable="$2"
    message="$3"
    cd "$directory"
    chmod +x "$executable"
    echo "============ Executing $message ==========="
    for i in 1 10 100 1000 10000
    do
        points=$((min_points * i))
        echo "CHAMFER Distance for $points Points"
        time ./"$executable" $points
        echo "================= ======================== ================="
    done
    cd ..
}

## Execute each case

execute_cd "baseline" "CD_baseline" "Baseline Case"
execute_cd "baseline_omp" "CD_baseline_omp" "Baseline Case with OpenMP"
execute_cd "baseline_GPU" "CD_baseline_GPU" "Baseline Case with GPU"
execute_cd "kd" "CD_kd" "KD Case"
execute_cd "kd_omp" "CD_kd_omp" "KD Case with Open MP"
