CRAZYFLIE_BASE := ../crazyflie-firmware

EXTRA_CFLAGS += -I$(PWD)/src
EXTRA_CFLAGS += -I$(PWD)/src/ls-slam
EXTRA_CFLAGS += -I$(PWD)/src/tof-matrix
EXTRA_CFLAGS += -I$(PWD)/src/tof-matrix/drivers
EXTRA_CFLAGS += -I$(PWD)/src/icp
EXTRA_CFLAGS += -I$(PWD)/src/flash
EXTRA_CFLAGS += -Wno-error

OOT_CONFIG := $(PWD)/app-config

include $(CRAZYFLIE_BASE)/tools/make/oot.mk

