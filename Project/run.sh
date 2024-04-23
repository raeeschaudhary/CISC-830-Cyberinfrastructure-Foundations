#!/bin/bash

## Perform Data Generation

## Compile Data Generator 

echo "================= ======================== ================="
echo "================= Compiling Data Generator ================="
echo "================= ======================== ================="
cd data_generator
cmake . 
make 
cd ..

## Execute Data Generator Generate Data Points

cd data_generator 
chmod +x run.sh
./run.sh 
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


# Execute Baseline Case

cd baseline 
chmod +x run.sh
./run.sh
cd ..

## Execute Baseline Case with Open MP

cd baseline_omp
chmod +x run.sh
./run.sh
cd ..

## Execute Baseline Case with GPU

cd baseline_GPU
chmod +x run.sh
./run.sh
cd ..

## Execute KD Case

cd kd
chmod +x run.sh
./run.sh
cd ..

## Execute KD Case with OpenMP

cd kd_omp
chmod +x run.sh
./run.sh
cd ..