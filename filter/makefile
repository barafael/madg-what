CFLAGS = -Wall -ansi -pedantic -std=c11

lib%.so: %.c
	gcc -shared -fPIC -lm $(CFLAGS) -o $@ $<

clean:

$(lib): madgwick_filter.h

