#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

static void sighandler(int s);
static int parse_args(int argc, char **argv);

static const char *rom_fname = "rom";
static const char *termdev = "/dev/tty";

static int ttyfd;
static struct termios saved_term;
static volatile int quit;

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
	term.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	term.c_oflag &= ~OPOST;
	term.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	term.c_cflag = (term.c_cflag & ~(CSIZE | PARENB)) | CS8;
	term.c_cc[VMIN] = 0;
	term.c_cc[VTIME] = 1;
	tcsetattr(ttyfd, TCSAFLUSH, &term);

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

		res = select(maxfd + 1, &rdset, 0, 0, &tv);
		if(quit) break;

		if(res > 0) {
			if(FD_ISSET(ttyfd, &rdset)) {
				int i, rd;
				char buf[256];

				while((rd = read(ttyfd, buf, sizeof buf)) > 0) {
					for(i=0; i<rd; i++) {
						emu_serin(0, buf[i]);
					}
				}
				if(!rd) break;	/* EOF */
			}
		}

		emu_step();
		sched_yield();
	}

	emu_cleanup();

	tcsetattr(ttyfd, TCSAFLUSH, &saved_term);
	return 0;
}

void emu_serout(int port, int c)
{
	write(ttyfd, &c, 1);
}

static void sighandler(int s)
{
	quit = 1;
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
