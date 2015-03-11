#ifndef SMOOTH_TRAJ_MANAGER_H
#define SMOOTH_TRAJ_MANAGER_H

#include "control_system.h"

#define SMOOTH_TRAJ_UPDATE_PERIOD_S 0.1 // 100 ms

#define SMOOTH_TRAJ_MAX_NB_POINTS 50

#define SMOOTH_TRAJ_DEFAULT_PRECISION_D_MM  40.0
#define SMOOTH_TRAJ_DEFAULT_PRECISION_A_RAD (5.0f * 0.01745329f) //5 degrees

#define SMOOTH_TRAJ_STEER_DISTANCE_MM 70


void smooth_traj_init();

void smooth_traj_start();

/* Remove every points from the trajectory */
void smooth_traj_end();
/* Check whether points remain in the trajectory*/
int smooth_traj_is_ended();

void smooth_traj_next_point();

uint32_t smooth_traj_get_cur_id();
uint32_t smooth_traj_get_last_id();

void smooth_traj_pause();
void smooth_traj_resume();

void smooth_traj_goto_xy_mm(float x, float y);

/*avoid to use that*/
void smooth_traj_goto_d_mm(float d);
void smooth_traj_goto_a_deg(float a);

#endif
