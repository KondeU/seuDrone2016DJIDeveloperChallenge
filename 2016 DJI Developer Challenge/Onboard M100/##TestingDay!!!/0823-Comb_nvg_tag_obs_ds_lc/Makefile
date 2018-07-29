CC=g++
TARGET = test
OBJECTS = test.o

INC = -I./DJIlib/inc -I./DJIlib/DJIscript/inc

APRILINC = -I./AprilTag -I./AprilTag/common

LIBAPRILTAG := libapriltag.a

LIBONBOARD := libonboard.a

LIBGUIDANCE := libguidance.a

CFLAGS = -Wall -O3 `pkg-config --cflags opencv` -I/usr/local/include/opencv2 -g -Wall -I/usr/local/include -DHAVE_OPENCV
LDFLAGS = -Wl,-rpath,./ -ldcam -lpthread -lrt -L./ -L/usr/local/lib/ `pkg-config --libs opencv`
GUIDANCELDFLAGS = -Wl,-rpath,./ -lpthread -lrt -L./ -L/usr/local/lib/ -lDJI_guidance -lusb-1.0 `pkg-config --cflags --libs opencv` -lopencv_core -lopencv_highgui #-l**DJI\_guidance**

all: $(TARGET) $(LIBAPRILTAG)

$(TARGET) : $(OBJECTS)  AprilTag/getcor.o mainapril.o mainonboard.o main_guidance.o $(LIBONBOARD) $(LIBAPRILTAG) $(LIBGUIDANCE)
	$(CC) -o $(TARGET) $(OBJECTS) $(LDFLAGS) AprilTag/getcor.o mainapril.o mainonboard.o main_guidance.o -lm `pkg-config --libs opencv` $(LIBONBOARD) $(LIBAPRILTAG) $(LIBGUIDANCE) $(GUIDANCELDFLAGS)-ldcam 

$(LIBAPRILTAG): 
	(cd ./AprilTag;$(MAKE))

$(LIBONBOARD): 
	(cd ./DJIlib;$(MAKE))

$(LIBGUIDANCE): 
	(cd ./guidance;$(MAKE))

$(OBJECTS): test.cpp $(LIBONBOARD)
	$(CC) $(CFLAGS) -c test.cpp -std=c++11 $(INC) -pthread -I./DJIlib/conboardSDK

AprilTag/getcor.o: AprilTag/getcor.cpp AprilTag/apriltag_demo.h AprilTag/getcor.h
	$(CC) -c AprilTag/getcor.cpp `pkg-config --cflags opencv`
	mv ./getcor.o ./AprilTag/getcor.o

mainapril.o: mainapril.cpp
	$(CC) -c mainapril.cpp -I/usr/include/opencv2 -ldcam -I./AprilTag 

mainonboard.o: mainonboard.cpp $(addprefix objs/, $(OBJS)) AprilTag/getcor.h
	$(CC) $(CFLAGS) -c mainonboard.cpp $(INC) -pthread -I./DJIlib/conboardSDK -I./AprilTag

main_guidance.o: main_guidance.cpp
	$(CC) -c  main_guidance.cpp $(CFLAGS) $(INC) -I./guidance

.PHONY: clean
	
clean:
	rm -f *.o *~ test $(LIBAPRILTAG) $(LIBONBOARD) $(LIBGUIDANCE)
