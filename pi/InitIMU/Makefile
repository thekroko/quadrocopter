# You need to "sudo apt-get install libgtkmm-3.0-dev" to build the demo_3d binary

all: initIMU

HDRS = helper_3dmath.h I2Cdev.h MPU6050_6Axis_MotionApps20.h MPU6050.h
CMN_OBJS = I2Cdev.o MPU6050.o
DMP_OBJS =  initIMU.o
CXXFLAGS = -DDMP_FIFO_RATE=3 -Wall -g -O2 `pkg-config gtkmm-3.0 --cflags --libs`

# = 50.0hz
# This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
# 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
# DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))

$(CMN_OBJS) $(DMP_OBJS) : $(HDRS)

initIMU: $(CMN_OBJS) $(DMP_OBJS)
	$(CXX) -o ../$@ $^ -lm

clean:
	rm -f $(CMN_OBJS) $(DMP_OBJS) $(D3D_OBJS) $(RAW_OBJS) ../initIMU

