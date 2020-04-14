#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

enum {NORMAL, OBSTACLE, ATTAQUE, INTERSECTION};
enum dir {RIGHT, FRONT, LEFT, BACK};

//start the PI regulator thread
void pi_regulator_start(void);


#endif /* PI_REGULATOR_H */
