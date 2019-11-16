# AvionicsTeam2020
This is the avionics for team 2020. It contains the software for the microcontrollers like the FSM, power systems, sensors etc. and also the Altium files for the PCBs.

## Structure
This repository is divided into subfolders. There is one folder for each microcontroller and the software it's gonna run. Each of the microcontroller folders will have a typical C/C++ structure for dividing up the code. That means "inc" has the header files (.h/.hpp), "src" has the source files (.c/.cpp) and "main.c" resides at the top level. If there are test modules of code during the development process, these files should have the structure of a "main.c" and reside in a folder called "test". In the folder "vscode" a workspace for Visual Studio Code will be placed. This is a workspace you can open in VScode to get access to all the folders and files for that project and easily do software development. The workspace also saves settings for formatting and other things. If VScode is used and you are developing on the same project use the "liveshare" extension in VScode to edit at the same time.

This project will involve several microcontrollers where one of them will be written in embedded C++ and the others in embedded C. Because of this the file extensions for the C files will be .h and .c, while the C++ files will be .hpp and .cpp.

**The folder structure works in the following way:**

- **AvionicsTeam2020** - Root folder
  - **Finite State Machine** - Subfolder
    - inc
    - src
    - test
    - vscode
    - main.cpp
  - **Sensors** - Subfolder
    - inc
    - src
    - test
    - vscode
    - main.c
  - **Engine** - Subfolder
    - inc
    - src
    - test
    - vscode
    - main.c
  - **Power systems** - Subfolder
    - inc
    - src
    - test
    - vscode
    - main.c
  - **Communications** - Subfolder
    - inc
    - src
    - test
    - vscode


![Folders](Images/folder_structure) 
