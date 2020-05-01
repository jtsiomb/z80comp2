Z80 computer v2
===============

About
-----
This is a simple Z80 computer. Hopefully I'll find a better name for it.
 - 3.6864 MHz clock (see mc68681 DUART)
 - 64kb RAM (32kb accessible at any single time)
 - 32kb ROM
 - Dual serial UART (mc68681) with CTS/RTS flow control (1x RS232, 1x TTL-level)
 - Glue logic implemented in a ATF16V8 GAL (glue.pld)

Directory structure:
 - `hw/`: hardware design files, schematic, and GAL equations
 - `rom/`: source code of the ROM monitor program
 - `emu/`: system emulator

License
-------
Copyright (C) 2020 John Tsiombikas <nuclear@member.fsf.org>

Hardware designs are open hardware, released under the terms of the Creative
Commons Attribution Share-Alike license (CC BY-SA). See `LICENSE.hw` for
details.

All software and firmware in this project is free software, released under the
terms of the GNU General Public License v3, or at your option any later version
published by the Free Software Foundation. See `LICENSE.sw` for details.

In short you are free to use, reproduce, modify, and/or redistribute any part of
this project, provided you also extend the same freedom, under the same terms,
to anyone receiving this, or any derivative work based on this, from you.
