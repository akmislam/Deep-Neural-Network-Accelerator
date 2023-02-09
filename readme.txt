Akm Islam
ESE 507
Project 3 - Readme File

--- Description of Project ---
This project is a hardware generator for a deep neural network accelerator.

A neural network is simply a set of inputs (vector) that are crossed with a set of trained weights (matrix) to produce an output (vector).
This can be represented by a matrix-vector multiplier.

Though we know what the design should be able to do, we can't be sure of its precise design specifications. That is, for different applications, we may have a different number of inputs coming in to cross with a different number of weights to produce a different number of outputs. To design a solution that fits one precise design specification would only solve that specific problem.

This project aims to design a generator that will create a neural network interface given the users' desired design specifications. I can tell the system I want X inputs with W weights and Y outputs, and the design will produce a correct solution everytime.


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
