SUB_DIRS = lx9_dave godil_250 godil_500 lx9_jason lx9_jason_flipped

build:
	for dir in $(SUB_DIRS); do \
	    $(MAKE) -C $$dir build; \
	done;

clean:
	for dir in $(SUB_DIRS); do \
	    $(MAKE) -C $$dir clean; \
	done;

clobber:
	for dir in $(SUB_DIRS); do \
	    $(MAKE) -C $$dir clobber; \
	done;
