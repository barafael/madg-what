CFLAGS = -Wall -ansi -pedantic -std=c11

.PHONY: all clean shared_obj

c_files := $(wildcard *.c)
objects := $(patsubst %.c,lib%.so,$(c_files))

all: shared_obj
	rm -rf *.o

shared_obj: $(objects)
	gcc -shared -fPIC -lm $(CFLAGS) $(c_files) -c

lib%.so: %.c
	gcc -shared -fPIC -lm $(CFLAGS) -o $@ $<

clean:
	rm -f *.so
	rm -f *.o
