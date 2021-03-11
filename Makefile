EXEC = gw-ev
OBJS = gw-ev.o
CC=gcc
#LDLIBS += -lcurl

CFLAGS = -Wall -g -O2


all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)
	

restart: all
	systemctl restart $(EXEC)
clean:
	-rm -f  *.elf *.gdb *.o




