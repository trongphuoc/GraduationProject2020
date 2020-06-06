LIBS=/opt/libjpeg-turbo/lib32/libturbojpeg.a

TARGET=de1_vip_1

CC = g++

LIBPATH =/opt/opencv_arm/lib

LIBRARIES = -lopencv_core -lopencv_videoio -lopencv_objdetect -lopencv_imgproc -lopencv_videoio -lopencv_video -lopencv_imgcodecs -lopencv_highgui

INCLUDE=-I. -I/opt/opencv_arm/include/opencv4 -I/opt/opencv_arm/include/opencv4/opencv2 -I/opt/opencv_arm/include/opencv4/opencv2/core

all:
	gcc capture1.c -o de1_vip /opt/libjpeg-turbo/lib32/libturbojpeg.a -I .
c_plus:
	$(CC) capture1.cpp -o $(TARGET) -L$(LIBPATH) $(LIBRARIES) $(LIBS) $(INCLUDE) 
clean: 
	rm *.out
distclean:
	rm *.jpg *.png
