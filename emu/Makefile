src = $(wildcard src/*.c)
obj = $(src:.c=.o)
dep = $(obj:.o=.d)
bin = z80v2emu

CFLAGS = -pedantic -Wall -g -MMD

$(bin): $(obj)
	$(CC) -o $@ $(obj) $(LDFLAGS)

-include $(dep)

.PHONY: clean
clean:
	rm -f $(obj) $(bin)
