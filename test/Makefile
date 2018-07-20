include ../../env.mk

INC = -I../uapi/ -I../../dv-user-driver/include

CFLAGS = -O0 -g -Wall -c $(INC)
LFLAGS = -O0 -g

OBJS = test.o
TGT  = test

%.o: %.c
	$(GCC) $(CFLAGS) -o $@ $<

all : $(OBJS)
	$(GCC) $(LFLAGS) $(OBJS) -o $(TGT)

clean:
	rm *.o
	rm -f $(TGT)
