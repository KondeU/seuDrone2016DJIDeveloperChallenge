LIBAPRILTAG := libapriltag.a

CC = gcc

AR = ar

CFLAGS = -std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function -pthread -I. -Icommon -O4
LDFLAGS = -lpthread -lm

APRILTAG_OBJS = apriltag.o apriltag_demo.o apriltag_quad_thresh.o tag16h5.o tag25h7.o tag25h9.o tag36h10.o tag36h11.o tag36artoolkit.o g2d.o common/zarray.o common/zhash.o common/zmaxheap.o common/unionfind.o common/matd.o common/image_u8.o common/pnm.o common/image_f32.o common/image_u32.o common/workerpool.o common/time_util.o common/svd22.o common/homography.o common/string_util.o common/getopt.o

all: $(LIBAPRILTAG)

$(LIBAPRILTAG): $(APRILTAG_OBJS)
	@echo "   [$@]"
	@ar -cq $@ $(APRILTAG_OBJS)
	mv ./$(LIBAPRILTAG) ../$(LIBAPRILTAG)

apriltag_demo.o:apriltag_demo.c apriltag_demo.h getcor.h
	g++ -c apriltag_demo.c `pkg-config --cflags opencv` -I./ -I./common $(CFLAGS)

%.o: %.c
	@echo "   $@"
	@$(CC) -o $@ -c $< $(CFLAGS)

clean:
	@rm -rf *.o common/*.o $(LIBAPRILTAG)

