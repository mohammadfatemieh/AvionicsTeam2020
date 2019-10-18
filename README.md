# AvionicsTeam2020
This is the avionics for team 2020. It contains the software for the microcontrollers like the FSM, power systems, sensors etc. and also the Altium files for the PCBs.

## Structure
This repository is divided into subfolders. There is one folder for each microcontroller and the software it's gonna run. Each of the microcontroller folders will have a typical C/C++ structure for dividing up the code. That means "inc" has the header files (.h/.hpp), "src" has the source files (.c/.cpp) and "main.c" resides at the top level. If there are test modules of code during the development process, these files should have the structure of a "main.c" and reside in a folder called "test".

This project will involve several microcontrollers where one of them will be written in embedded C++ and the others in embedded C. Because of this the file extensions for the C files will be .h and .c, while the C++ files will be .hpp and .cpp.
