CFLAGS = -Wall -ansi -pedantic -std=c11

c_files := $(wildcard *.c)
objects := $(patsubst %.c,lib%.so,$(wildcard *.c))

all: $(objects)
	gcc -shared -fPIC -lm $(CFLAGS) $(c_files) -c

lib%.so: %.c
	gcc -shared -fPIC -lm $(CFLAGS) -o $@ $<

clean:
	rm -f *.so
