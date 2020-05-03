src = monitor.asm
bin = monitor

romtype = 'AT28C256'
romsize = 2048

AS = vasmz80_oldstyle
ASFLAGS = -Fbin

$(bin): monitor.asm
	$(AS) -o $@ $(ASFLAGS) -L monitor.lst $< >/dev/null

.PHONY: clean
clean:
	rm -f $(bin)

.PHONY: program
program: $(bin)
	minipro -p 'AT28C256E' -s -w $<
