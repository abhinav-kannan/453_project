453_project
===========

EECS 453 Project
Goal: To study the feasibility of Amdahl's law on multicore processor architectures by evaluating existing benchmarks

Build Environment:
  ALPHA_FS mode
  CPU Models AtomicSimple, TimingSimple O3CPU InOrder
  gem5.opt binary

EECS 453 Project files

This project contains the files which need to be added/modified in Gem5 in order to run the project.

Files added:
1. new_fs.py 
  Script which is used to run the simulations
  Options added:
    i) --num-bce to specify the number of base core equivalents
    ii) --num-r to specify the number of resources per core
    iii) --asymmetric to toggle between symmetric and asymmetric modes

Files modified:
1. CacheConfig.py
  Added a separate method to configure the cache hierarchy for the new architecture

This readme maybe incomplete. Please check the code for further details.