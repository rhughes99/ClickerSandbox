--- Clicker Decoder Sandbox Notes

Playing around with the BBB and a TSOP38238 IR Receiver Module
Started with ancient Magnavox TV/VCR controller, and switched to TV (small)
  controller in June 2021.
The Controller prints codes it doesn't know, so you can teach it new tricks.

(Ins and Outs relative to BBB)
P9_31 = IR input
P9_29 = LED
P9_28 = a toggled test point

========================================
To Run:
source setup.sh				- to set some parameters and configure I/O pins
gcc ClickSandController.c -o Controller	- to compile Controller
make					- to compile PRU code and install and start
./Controller				- to start Controller
