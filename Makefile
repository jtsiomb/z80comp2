glue.jed: glue.pld

%.jed: %.pld
	galasm $<

.PHONY: program
program: glue.jed
	minipro -p ATF16V8B -c config -w $<
