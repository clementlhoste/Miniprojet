#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

enum {NORMAL, OBSTACLE, ATTAQUE, INTERSECTION, CHOIX_CHEMIN};
enum dir {RIGHT, FRONT, LEFT, BACK};
enum {DEMO1, DEMO2};
#define CONV_CM2STEP 1000/13 // MAGIC NB

//start the PI regulator thread
void rob_management_start(void);


#endif /* PI_REGULATOR_H */
