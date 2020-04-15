#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

enum {NORMAL, OBSTACLE, ATTAQUE, INTERSECTION, CHOIX_CHEMIN};
enum dir {RIGHT, FRONT, LEFT, BACK};

#define CONV_CM2STEP 1000/13 // MAGIC NB

//start the PI regulator thread
void pi_regulator_start(void);


#endif /* PI_REGULATOR_H */
