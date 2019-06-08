# quiet.
Q = @

PWD = $(shell pwd)

# gcc compiler tools.
CC = gcc
LD = gcc
RM = rm

# target
TARGET = demo.bin

# source files
SRCS = main.c
OBJS = main.o

# header dir
INCLUDE = -I$(PWD)
LIB = -lrtcm

# cflags
CFLAGS = -Wall -O2 
LDFLAG = -L. -Wl,-rpath=. -std=c99

.PHONY: all
all:
	$(Q) $(CC) -o $(TARGET) $(SRCS) $(CFLAGS) $(LIB) $(LDFLAG)
	$(Q) echo "make $(TARGET) done!!!"

clean:
	$(Q) $(RM) -f $(OBJS) $(TARGET)
	$(Q) echo "clean done!!!"

test:
	echo $(PWD)

