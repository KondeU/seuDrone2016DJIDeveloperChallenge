#2016-07-01

LIBGUIDANCE := libguidance.a

CXX = g++
AR = ar

CFLAGS = -g -Wall -I/usr/local/include -I../../../include -DHAVE_OPENCV -I/usr/local/include/opencv2

LDFLAGS = -Wl,-rpath,./ -lpthread -lrt -L./ -L/usr/local/lib/ -lDJI_guidance -lusb-1.0 `pkg-config --cflags --libs opencv` -lopencv_core -lopencv_highgui

GUIDANCE_OBJS = DJI_utility.o #main_guidance.o

all: $(LIBGUIDANCE)

$(LIBGUIDANCE): $(GUIDANCE_OBJS)
	@echo "   [$@]"
	@ar -cq $@ $(GUIDANCE_OBJS)
	mv ./$(LIBGUIDANCE) ../$(LIBGUIDANCE)

%.o : %.cpp
	@echo "   $@"
	@$(CXX) -o $@ -c $< $(CFLAGS)

clean :
	- rm -r -f ./*.o  $(LIBGUIDANCE)
