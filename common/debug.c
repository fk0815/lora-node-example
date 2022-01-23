/*
Copyright © 2022 Frank Kunz

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <debug.h>
#include <stdio.h>

#ifdef DEBUG_SEMIHOST
extern void initialise_monitor_handles(void);
void debug_printf_init(void)
{
	initialise_monitor_handles();
}
#define _write write
#include <unistd.h>
#include <lmic.h>
static const char *ev_txt[] = { LMIC_EVENT_NAME_TABLE__INIT };
#define ev_txt_count (sizeof(ev_txt)/sizeof(ev_txt[0]))
void debug_hexdump(char *tag, void *buf, unsigned len)
{
	char ascii_line[20];
	char hex_line[52];
	char offset_line[20];
	unsigned i, i_h, i_a;
	char *p = buf;
	if(buf && len) {
		i_h = 0;
		i_a = 0;
		memset(ascii_line, 0, sizeof(ascii_line));
		memset(hex_line, ' ', sizeof(hex_line));
		snprintf(offset_line, sizeof(offset_line), "[0000]");
		for(i=0;i<len;i++) {
			if(i && !(i%16)) {
				/* print lines */
				if(tag)
					_write(1, tag, strlen(tag));
				_write(1, offset_line, strlen(offset_line));
				_write(1, hex_line, 49);
				_write(1, " | ", 3);
				_write(1, ascii_line, strlen(ascii_line));
				_write(1, "\n", 1);
				i_h = 0;
				i_a = 0;
				memset(ascii_line, 0, sizeof(ascii_line));
				memset(hex_line, ' ', sizeof(hex_line));
				snprintf(offset_line, sizeof(offset_line), "[%04x]", i);
			} else if(i && !(i%8)) {
				/* add spacer */
				hex_line[i_h++] = ' ';
			}
			i_h += snprintf(hex_line + i_h, sizeof(hex_line) - i_h - 1, " %02x", (uint8_t)p[i]);
			hex_line[i_h] = ' ';
			ascii_line[i_a++] = (p[i] < 32)?('.'):((p[i] > 127)?('.'):(p[i]));
		}
		if(tag)
			_write(1, tag, strlen(tag));
		_write(1, offset_line, strlen(offset_line));
		_write(1, hex_line, 49);
		_write(1, " | ", 3);
		_write(1, ascii_line, strlen(ascii_line));
		_write(1, "\n", 1);
	}
}
void debug_print_event(uint8_t e)
{
	if(e < ev_txt_count)
		printf("%s\n", ev_txt[e]);
}
#endif
