#ifndef UTILITIES_H
#define UTILITIES_H


// Configure pinModes
void setupPins();

// 'F', 'D', 'I' for fill, dump, air; smaller case to close valves
void valveControl(char valve);

// 'A', 'O' for arm and fire; smaller case to stop
void ignition(char order);  

// Check = 'S' to print out states;
void statusCheck(char check);


#endif /* UTILITIES_H */
