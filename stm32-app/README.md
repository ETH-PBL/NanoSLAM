# Onboard SLAM with Nano-UAVs
This code enables running the Least Squares SLAM (LS-SLAM) on a Crazyflie 2.1
## Instructions
The code is developed to run on both a linux computer and the nano-drone. For debugging purposes, running it on the computer might be preferred.
### Running the code on the computer
`cd app-ls-slam-crazyflie/src`
`make all run`
### Running the code on the drone
`cd app-ls-slam-crazyflie`
`make all`
`make cload` -- with the drone in bootloader mode

Documentation in progress:
graph-based-slam.h

