#ifndef ROBOT_MANAGEMENT_H
#define ROBOT_MANAGEMENT_H

// Enum to define the different states of an FSM in which the robot can be
enum {NORMAL, OBSTACLE, ATTAQUE, INTERSECTION, CHOIX_CHEMIN, DEMI_TOUR, END};

// Enum to define the different directions the robot can take in an intersection
// and in which direction it is checking
enum {RIGHT, FRONT, LEFT, BACK};

// Enum to define the different demo modes
//DEMO1 : line alignment, robot moving through the maze, displacement algorithm, management of obstacles as walls
//DEMO2 : same but now the robot can go through obstacles like a battering ram
//DEMO3 : the synthesis of DEMO1/2, robot is using voice recognition to deal with obstacles using go/back command
enum {DEMO1, DEMO2, DEMO3, NB_DEMOS};

/*
*	Starts the Robot Management Thread
*/
void rob_management_start(void);

#endif /* ROBOT_MANAGEMENT_H */
