#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <sched.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/mman.h>
#include "emu.h"
#include "cpu.h"
#include "dbg.h"

static int cmd_input(char *line);
static void print_prompt(void);
static void print_flags(void);
static int term_raw(void);
static int term_cooked(void);
static void sighandler(int s);
static int parse_args(int argc, char **argv);

static const char *rom_fname = "rom";
static const char *termdev = "/dev/tty";

static int ttyfd;
static FILE *ttyfile;
static struct termios saved_term;
static volatile int quit;

static int cmdmode;

int main(int argc, char **argv)
{
	void *rom;
	int res, fd, maxfd;
	struct stat st;
	struct termios term;
	fd_set rdset;
	struct timeval tv = {0, 0};

	if(parse_args(argc, argv) == -1) {
		return 1;
	}

	if((fd = open(rom_fname, O_RDONLY)) == -1) {
		fprintf(stderr, "failed to open ROM image: %s: %s\n", rom_fname, strerror(errno));
		return -1;
	}
	fstat(fd, &st);

	if((rom = mmap(0, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0)) == (void*)-1) {
		fprintf(stderr, "failed to map ROM image\n");
		return -1;
	}

	if((ttyfd = open(termdev, O_RDWR)) == -1) {
		fprintf(stderr, "failed to open terminal device: %s: %s\n", termdev, strerror(errno));
		return -1;
	}
	if(tcgetattr(ttyfd, &term) == -1) {
		perror("failed to get terminal attributes");
		return -1;
	}
	saved_term = term;
	if(!cmdmode) {
		term_raw();
	} else {
		opt_loginstr = 1;
		print_prompt();
	}

	ttyfile = fdopen(ttyfd, "w");
	dbg_log_file(ttyfile);

	if(emu_init(rom, st.st_size) == -1) {
		return 1;
	}

	signal(SIGINT, sighandler);
	signal(SIGQUIT, sighandler);
	signal(SIGTERM, sighandler);
	signal(SIGSEGV, sighandler);
	signal(SIGILL, sighandler);

	while(!quit) {
		FD_ZERO(&rdset);
		FD_SET(ttyfd, &rdset);

		maxfd = ttyfd;

		res = select(maxfd + 1, &rdset, 0, 0, cmdmode ? 0 : &tv);
		if(quit) break;

		if(res > 0) {
			if(FD_ISSET(ttyfd, &rdset)) {
				int i, rd;
				static char buf[4096];

				while((rd = read(ttyfd, buf, sizeof buf - 1)) > 0) {
					if(cmdmode) {
						char *line, *ptr;
						line = ptr = buf;
						while(ptr - buf < rd) {
							if(*ptr == '\n') {
								*ptr = 0;
								cmd_input(line);
								if(quit) goto end;
								if(!cmdmode) break;
								line = ptr + 1;
								print_prompt();
							}
							ptr++;
						}
					} else {
						for(i=0; i<rd; i++) {
							int c = buf[i];

							switch(c) {
							case 4:
								goto end;
							case 3:
								cmdmode = 1;
								opt_loginstr = 1;
								term_cooked();
								print_prompt();
								break;
							default:
								emu_serin(0, buf[i]);
							}
						}
					}
				}
			}
		}

		if(!cmdmode) {
			emu_step();
			sched_yield();
		}
	}
end:

	emu_cleanup();

	tcsetattr(ttyfd, TCSAFLUSH, &saved_term);
	return 0;
}

void emu_serout(int port, int c)
{
	write(ttyfd, &c, 1);
}

static int cmd_input(char *line)
{
	int i, line_len, count, argc = 0;
	char *argv[128];
	static char last_line[4096];
	static char *last_argv[128];
	static int last_argc;
	struct registers *regs;
	uint16_t addr;

	line_len = strlen(line);
	while(argc < 128 && (argv[argc] = strtok(argc ? 0 : line, " \t\n\r"))) {
		argc++;
	}

	if(!argc) {
		line = last_line;
		memcpy(argv, last_argv, sizeof argv);
		argc = last_argc;
	}

	switch(argv[0][0]) {
	case 'q':
		quit = 1;
		break;

	case 'c':
		cmdmode = 0;
		term_raw();
		break;

	case 's':
		emu_step();
		break;

	case 'r':
		regs = cpu_regs();
		printf("af: %02x %02x [", (unsigned int)regs->g.r.a, (unsigned int)regs->g.r.f);
		print_flags();
		printf("]\n");
		printf("bc: %02x %02x\n", (unsigned int)regs->g.r.b, (unsigned int)regs->g.r.c);
		printf("de: %02x %02x\n", (unsigned int)regs->g.r.d, (unsigned int)regs->g.r.e);
		printf("hl: %02x %02x\n", (unsigned int)regs->g.r.h, (unsigned int)regs->g.r.l);
		printf("ix: %04x iy: %04x\n", (unsigned int)regs->ix, (unsigned int)regs->iy);
		printf("sp: %04x pc: %04x\n", (unsigned int)regs->sp, (unsigned int)regs->pc);
		printf("iff: %02x imode: %d\n", (unsigned int)regs->iff, (int)regs->imode);
		break;

	case 'x':
		if(argv[0][1] == '/') {
			if((count = atoi(argv[0] + 2)) <= 0) {
				fprintf(stderr, "invalid byte count: %d\n", count);
				break;
			}
		} else {
			count = 1;
		}
		if(argc > 1) {
			char *endp;
			addr = strtol(argv[1], &endp, 0);
			if(endp == argv[1]) {
				int val = cpu_get_named(argv[1]);
				if(val < 0) {
					fprintf(stderr, "invalid address: %s\n", argv[1]);
					break;
				}
				addr = val;
			}
		} else {
			addr = cpu_regs()->pc;
		}
		dbg_mem_dump(addr, count);
		break;

	default:
		fprintf(stderr, "unrecognized command: %s\n", argv[0]);
		break;
	}

	if(line != last_line) {
		memcpy(last_line, line, line_len + 1);
		for(i=0; i<argc; i++) {
			last_argv[i] = last_line + (argv[i] - line);
		}
		last_argc = argc;
	}
	return 0;
}

static void print_prompt(void)
{
	struct registers *regs = cpu_regs();
	printf("%04x [a:%02x f:", (unsigned int)regs->pc, (unsigned int)regs->g.r.a);
	print_flags();
	printf("]> ");
	fflush(stdout);
}

struct {
	char c;
	unsigned int mask;
} flagbits[] = {
	{'s', FLAGS_S},
	{'z', FLAGS_Z},
	{'h', FLAGS_H},
	{'p', FLAGS_PV},
	{'n', FLAGS_N},
	{'c', FLAGS_C},
	{0, 0}
};

static void print_flags(void)
{
	int i;
	struct registers *regs = cpu_regs();
	unsigned int flags = regs->g.r.f;

	for(i=0; flagbits[i].c; i++) {
		int c = flagbits[i].c;
		if(flags & flagbits[i].mask) {
			c = toupper(c);
		}
		putchar(c);
	}
}

static int term_raw(void)
{
	struct termios term;

	if(tcgetattr(ttyfd, &term) == -1) {
		perror("failed to get terminal attributes");
		return -1;
	}
	term.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	term.c_oflag &= ~OPOST;
	term.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	term.c_cflag &= ~(CSIZE | PARENB);
	term.c_cflag |= CS8;
	term.c_cc[VMIN] = 0;
	term.c_cc[VTIME] = 1;
	tcsetattr(ttyfd, TCSAFLUSH, &term);
	return 0;
}

static int term_cooked(void)
{
	/*
	struct termios term;

	if(tcgetattr(ttyfd, &term) == -1) {
		perror("failed to get terminal attributes");
		return -1;
	}
	term.c_iflag |= IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON;
	term.c_oflag |= OPOST;
	term.c_lflag |= ECHO | ECHONL | ICANON | ISIG | IEXTEN;
	term.c_cflag |= CSIZE | PARENB;
	term.c_cflag &= ~CS8;
	tcsetattr(ttyfd, TCSAFLUSH, &term);
	*/
	tcsetattr(ttyfd, TCSAFLUSH, &saved_term);
	return 0;
}

static void sighandler(int s)
{
	signal(s, sighandler);
	if(s == SIGINT) {
		cmdmode = 1;
	} else {
		quit = 1;
	}
}

static int parse_args(int argc, char **argv)
{
	int i;

	for(i=1; i<argc; i++) {
		if(argv[i][0] == '-') {
			if(argv[i][2] == 0) {
				switch(argv[i][1]) {
				case 'r':
					if(!argv[++i]) {
						fprintf(stderr, "-r must be followed by a rom image file\n");
						return -1;
					}
					rom_fname = argv[i];
					break;

				case 'd':
					opt_loginstr = 1;
					break;

				case 'c':
					cmdmode = 1;
					break;

				default:
					fprintf(stderr, "invalid option: %s\n", argv[i]);
					return -1;
				}
			} else {
				fprintf(stderr, "invalid option: %s\n", argv[i]);
				return -1;
			}
		} else {
			fprintf(stderr, "unexpected argument: %s\n", argv[i]);
			return -1;
		}
	}
	return 0;
}
