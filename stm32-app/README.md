# NanoSLAM: The STM32 Application
This code represents the application that runs on the STM32 microcontroller soldered on the drone's main board. It is in charge of the sensor data acquisition and manages the communication with the GAP9 SoC. 
The STM32 not only accommodates our custom application but also the open source base firmware which is responsible for flight control, state estimation, data logging, etc. Therefore, the base firmware is a prerequisite for compiling our software, which runs on top of the base firmware.
## Instructions
1. Install the open-source base firmware: https://github.com/bitcraze/crazyflie-firmware
When cloning the firmware, place it in the *NanoSLAM/* folder, at the same level with *stm32-app*
Note: we used the commit 5c0001692cd2b77913325765cc6a9fdc2a5f1fac
Since we can not ensure compatibility between our software and the future firmware releases, you can optionally use the same commit:
`cd crazyflie-firmware`
`git checkout 5c0001692cd2b77913325765cc6a9fdc2a5f1fac`

2. Copy the files *estimator_kalman.patch* and *estimator_kalman2.patch* from this repository into the *crazyflie-firmware/* folder. Apply the patches:
`git apply < estimator_kalman.patch`
`git apply < estimator_kalman.patch`

3. Go to this folder: `cd ../stm32-app`
4. Compile the application: `make clean all -j8`
5. Put the drone in the bootloader mode by turning it off and then holding the power button until the blue leds alternatively blink. Plug in the CrazyRadio USB dongle into the computer and flash the drone:
`make cload`
