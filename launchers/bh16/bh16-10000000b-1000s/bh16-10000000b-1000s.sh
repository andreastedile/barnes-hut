#!/bin/bash

#PBS -l select=16:ncpus=16:mem=4gb

#PBS -l walltime=0:10:00

#PBS -q short_cpuQ

module load gcc91
module load mpich-3.2
mpirun.actual -n 16 ./hpc/barnes-hut/build-release4/simulators/mpi-barnes-hut-simulator/mpi-barnes-hut-simulator 10000000-bodies.txt 1000 1 --no-output
