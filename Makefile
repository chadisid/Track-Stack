PROGNAME = trackstack
CFLAGS = -O2 -Wall -Werror -Wextra 
CFILES = $(PROGNAME).c

all:
	gcc $(CFILES) $(CFLAGS) `pkg-config --cflags opencv` `pkg-config --libs opencv` -o $(PROGNAME) 
