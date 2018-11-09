all: filters

filters:
	cd filter && $(MAKE) -k all

clean:
	cd filter && $(MAKE) clean
