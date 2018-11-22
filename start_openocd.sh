#!/bin/bash
echo "Starting OpenOCD..."

openocd -f openocd.cfg

#starts openocd. You can connect from gdb at the default port 3333
#in gdb run "target remote localhost:3333"
#alternatively use putty/telnet using raw tcp at port 4444,
#e.g. telnet localhost 4444
