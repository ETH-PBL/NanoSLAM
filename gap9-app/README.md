# NanoSLAM: The GAP9 Application
The GAP9 applications represents the heart of NanoSLAM and it is in charge of the scan matching and pose graph optimization. However, to compile the code, you must first install the GAP9 SDK. If you don't have access to it, you can contact [GreenWaves Technologies](https://greenwaves-technologies.com/gap9-docs/) since it is not public yet. After installing the SDK and sourcing the platform through the sourceme.sh file, you can compile the code by doing the following:
## Build and Flash the Code
1. Connect the JTAG to the computer
2. Go to this folder: `cd gap9-app`
3. Run the code:
```bash
make clean
make all
make run
```

## Running the Unit Tests
To ensure the proper functionality, several unit tests were provided with this software. Running the unit tests does not necessarily require the physical platform. They can run using the GVSOC virtual platform as follows:
1. Open the Makefile
2. Comment out the line `APP_SRCS = main.c`
3. Uncomment one of the lines 9, 10 or 11, depending on which unit test you want to run
4. Compile and run the code using the virtual platform
  
```bash
make clean
make all platform=gvsoc
make run platform=gvsoc
```
