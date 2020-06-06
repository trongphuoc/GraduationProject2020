#CC = g++
CC = arm-linux-gnueabihf-g++
TARGET = capture1.cpp
OUTPUT = de1_vip
TARGET_OUTPUT = de1_vip_cross

LIBPATH =/opt/opencv-arm/lib
LIBTURBOJPEG =/opt/libjpeg-turbo/lib32
JPEG_LIBRARIES =-lturbojpeg -ljpeg
LIBRARIES = -lopencv_core -lopencv_videoio -lopencv_objdetect -lopencv_imgproc -lopencv_videoio -lopencv_video -lopencv_imgcodecs -lopencv_highgui

INCLUDE = -I /opt/opencv-arm/include/opencv4 -I/opt/opencv-arm/include/opencv4/opencv2 -I/opt/opencv-arm/include/opencv4/opencv2/core -I/opt/libjpeg-turbo/include -I .

all: 
	$(CC) $(TARGET) -o $(TARGET_OUTPUT) -L$(LIBPATH) $(LIBRARIES) -L$(LIBTURBOJPEG) $(JPEG_LIBRARIES) $(INCLUDE)
#        $(CC) $(TARGET) -o $(OUTPUT) -L$(LIBPATH) $(LIBRARIES) -L$(LIBTURBOJPEG) $(JPEG_LIBRARIES) $(INCLUDE)

clean:
	rm -rf *.o
clean_all:
	rm -rf *.o *.jpg $(OUTPUT)
