Akm Islam
ESE 507
Project 3 - Readme File



--- Instructions on using files ---


1. run "make" in linux terminal to compile files.

2. run any of the 3 testmode scripts as described in the project description. For example:
./testmode1 4 4 12 1
./testmode2 16 8 17 1 4
./testmode3 4 7 5 3 8 1 5



--- Note ---
Make sure the include .sv file is in the same directory as produced HDL.

When running these files, you will get a warning about a missing connection to the control module 
(same as my project 2 code). This is safe to ignore. I use this port for the last instantiated
control unit to synchronize them together. 