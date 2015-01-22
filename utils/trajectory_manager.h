#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include "control_system.h"

#define TRAJECTORY_UPDATE_PERIOD_S 0.1 // 100 ms

#define TRAJECTORY_MAX_NB_POINTS 50

#define TRAJECTORY_DEFAULT_PRECISION_D_MM  10.0
#define TRAJECTORY_DEFAULT_PRECISION_A_DEG 5.0f //1.0


void trajectory_init();

void trajectory_start();

/* Remove every points from the trajectory */
void trajectory_end();
/* Check whether points remain in the trajectory*/
int trajectory_is_ended();

void trajectory_next_point();

uint32_t trajectory_get_cur_id();
uint32_t trajectory_get_last_id();

void trajectory_pause();
void trajectory_resume();

void trajectory_goto_d_mm(float d_mm);

/* Set absolute angle. Does not depend on current angle. */
void trajectory_goto_a_abs_deg(float a_deg_ref);

/* Set relative angle. Depends on current angle. */
void trajectory_goto_a_rel_deg(float a_deg);
#endif /* TRAJECTORY_MANAGER_H */
