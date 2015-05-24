#ifndef ACTIONS_H
#define ACTIONS_H

#include <stdint.h>

void init_actions(void);

void action_open_grip(void);
void action_half_open_grip(void);
void action_close_grip(void);

void action_raise_lift(void);
void action_lower_lift(void);

void action_open_clasp(void);
void action_close_clasp(void);
#endif //ACTIONS_H
