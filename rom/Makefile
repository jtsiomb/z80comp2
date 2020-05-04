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

monitor.rom: $(bin)
	dd if=/dev/zero of=$@ bs=1024 count=32
	dd if=$< of=$@ bs=1024 conv=notrunc

.PHONY: program
program: monitor.rom
	minipro -p 'AT28C256E' -s -w $<
