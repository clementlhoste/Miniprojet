#ifndef ROBOT_MANAGEMENT_H
#define ROBOT_MANAGEMENT_H

// Enum to define the different states of an FSM in which the robot can be
enum {NORMAL, OBSTACLE, ATTAQUE, INTERSECTION, CHOIX_CHEMIN, DEMI_TOUR, END};

// Enum to define the different directions the robot can take in an intersection
// and in which direction it is checking
enum {RIGHT, FRONT, LEFT, BACK};

// Enum to define the different demo modes
enum {DEMO1, DEMO2, DEMO3, NB_MODES};

#define CONV_CM2STEP		1000/13
#define MAX_STD_INTER		10.7f
#define MAX_STD_WHITE		15.0f

//start the PI regulator thread
void rob_management_start(void);

#endif /* ROBOT_MANAGEMENT_H */
