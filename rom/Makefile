src = monitor.asm
obj = $(src:.asm=.o)
bin = monitor

romtype = 'AT28C256'
romsize = 2048

AS = vasmz80_oldstyle
ASFLAGS = -Fvobj
LD = vlink
LDFLAGS = -b rawbin1

$(bin): $(obj)
	$(LD) -o $@ $(LDFLAGS) -Ttext 0 $(obj)

%.o: %.asm
	$(AS) -o $@ $(ASFLAGS) -L $*.lst $< >/dev/null

.PHONY: clean
clean:
	rm -f $(bin) $(obj)

.PHONY: program
program: $(bin)
	minipro -p 'AT28C256E' -s -w $<
