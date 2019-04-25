
SOURCE  := $(wildcard ./*.cpp)
#wildcard把指定目录 ./ 下的所有后缀是cpp的文件全部展开。
DIR     := $(notdir $(SOURCE))
#notdir把展开的文件去除掉路径信息
OBJS    := $(patsubst %.cpp,%.o,$(DIR))
#patsubst把$(DIR)中的变量符合后缀是.cpp的全部替换成.o，

# target you can change test to what you want
TARGET  := capture
$(CC) = gcc
$(CXX) = g++

#ifeq ($(CC),aarch64-linux-gcc)
#else ifeq ($(findstring arm-poky-linux-gnueabi-gcc,$(CC)), arm-poky-linux-gnueabi-gcc)

LDFLAGS := -L. -L$(SDKTARGETSYSROOT)/usr/lib 
INCLUDE := -I$(SDKTARGETSYSROOT)/usr/include/ -L./lib
LIBS    :=   -lm -lopencv_core -lopencv_highgui  -lopencv_features2d \
			-lopencv_imgproc -lopencv_xfeatures2d -lopencv_calib3d -lopencv_imgcodecs \
			-lopencv_flann -lopencv_videoio -ljpeg


DEFINES := -Wl,-rpath=./lib

CFLAGS  := -g -Wall $(DEFINES) $(INCLUDE)

CXXFLAGS:= $(CFLAGS)

all : $(TARGET)

$(TARGET) : $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS) $(LIBS)
	echo $(CC) $(TARGET) ok

objs : $(OBJS)

rebuild: clean everything

clean :
	rm -rf *.o
	rm -rf $(TARGET)

