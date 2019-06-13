
TARGET=demo.bin
#CXX=$(TOOLCHAIN_PATH)$(CROSS_COMPILE)g++
CC = arm-linux-gnueabi-gcc

#CC_FLAG = -Wl,-rpath=./libs -Llibs -Iinclude -fpermissive -lqxinertial -lqxrtk_hal -lpthread
CC_FLAG = -Wl,-rpath=./libs -Llibs -Iinclude -lqxinertial -lqxrtk_hal -lpthread

all:
	@$(CC) demo_main.c -o $(TARGET) $(CC_FLAG)


clean:
	@rm -rf $(TARGET)

.PHONY: all clean
