#ifndef STATES_H
#define STATES_H

enum state_t {
    PRE_FILL, FILL, POST_FILL, ARM, IGNITION, DUMP
};

void state_preFill();
void state_fill();
void state_postFill();
void state_arm();
void state_ignition();

// Configure pinModes
void setupPins();

// 'F', 'D', 'I' for fill, dump, air; smaller case to close valves
void valveControl(char valve);

// 'A', 'O' for arm and fire; smaller case to stop
void ignition(char order);

// Check = 'S' to print out states;
void statusCheck(char check);

#endif /* STATES_H */
