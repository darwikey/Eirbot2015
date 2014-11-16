#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "position_manager.h"
#include "trajectory_manager.h"

enum trajectory_order_type {
  PAUSE, D, A_ABS, A_REL
};

enum trajectory_when {
  NOW, END
};

struct trajectory_dest {
  union {
    struct {
      float mm;
      float precision;
    } d;

    struct {
      float deg;
      float precision;
    } a;
  };

  float starting_d_mm;
  float starting_a_deg;
  uint8_t is_init;
  enum trajectory_order_type type;
};


/******************** Global variable ********************/
struct trajectory_manager {
  struct trajectory_dest points[TRAJECTORY_MAX_NB_POINTS];
  uint32_t cur_id;
  uint32_t last_id;
} traj;


#define ABS(x) (((x) < 0)? -(x): (x))

/********************   Prototypes   ********************/
void trajectory_task(void *data);
static inline void trajectory_update();
static inline void trajectory_update_smoothly();
static void trajectory_add_point(struct trajectory_dest point, enum trajectory_when when);


/******************** User functions ********************/

void trajectory_init()
{
  traj.cur_id = 0;
  traj.last_id = 0;
}

void trajectory_start()
{
  xTaskCreate(trajectory_task, (const signed char *)"TrajectoryManager", 200, NULL, 1, NULL );
}

void trajectory_end()
{
  traj.cur_id = traj.last_id;
}

int trajectory_is_ended()
{
  return (traj.cur_id == traj.last_id);
}

void trajectory_next_point()
{
  /* Update list pointer if not empty */
  if (!trajectory_is_ended()) {
    traj.cur_id = (traj.cur_id+1) % TRAJECTORY_MAX_NB_POINTS;
  }
}

uint32_t trajectory_get_cur_id()
{
  return traj.cur_id;
}

uint32_t trajectory_get_last_id()
{
  return traj.last_id;
}

/******************** Movement functions ********************/

static int trajectory_is_paused()
{
  return (!trajectory_is_ended() &&
          (traj.points[traj.cur_id].type == PAUSE));
}

void trajectory_pause()
{
  if (!trajectory_is_paused()) {
      struct trajectory_dest dest;
      dest.type = PAUSE;
      trajectory_add_point(dest, NOW);
  }

  /* Force update now to stop more quickly */
  trajectory_update(traj);
}

void trajectory_resume()
{
  while (trajectory_is_paused()) {
    trajectory_next_point();
  }
}

void trajectory_goto_d_mm(float d_mm)
{
  struct trajectory_dest dest;

  dest.type = D;
  dest.d.mm = d_mm;
  dest.d.precision = TRAJECTORY_DEFAULT_PRECISION_D_MM;

  trajectory_add_point(dest, END);
}

/* Absolute angle = current angle + relative angle. */
void trajectory_goto_a_abs_deg(float a_deg)
{
  struct trajectory_dest dest;

  dest.type = A_ABS;
  dest.a.deg = a_deg;
  dest.a.precision = TRAJECTORY_DEFAULT_PRECISION_A_DEG;

  trajectory_add_point(dest, END);
}

void trajectory_goto_a_rel_deg(float a_deg)
{
  struct trajectory_dest dest;

  dest.type = A_REL;
  dest.a.deg = a_deg;
  dest.a.precision = TRAJECTORY_DEFAULT_PRECISION_A_DEG;

  trajectory_add_point(dest, END);
}

/****************** Internal functions ******************/

void trajectory_task(void *data)
{
  (void)data;

  for (;;) {
    trajectory_update();
    vTaskDelay(TRAJECTORY_UPDATE_PERIOD_S * 1000 / portTICK_RATE_MS);
  }
}

static inline int trajectory_is_full()
{
  return (((traj.last_id+1) % TRAJECTORY_MAX_NB_POINTS) == traj.cur_id);
}

static inline void trajectory_decrease_id(uint32_t *id)
{
  *id = (*id + TRAJECTORY_MAX_NB_POINTS - 1) % TRAJECTORY_MAX_NB_POINTS;
}

void trajectory_add_point(struct trajectory_dest point, enum trajectory_when when)
{
  point.is_init = 0;

  if (when == END) {
    if (trajectory_is_full()) {
      printf("[trajectory_manager] Warning: List of points is full. Last point not added.\n");
      return;
    }

    /* New points are added at the end of the list */
    traj.points[traj.last_id] = point;

    /* Update end of list pointer */
    traj.last_id = (traj.last_id + 1) % TRAJECTORY_MAX_NB_POINTS;
  }
  else if (when == NOW) {
    if (trajectory_is_full()) {
      trajectory_decrease_id(&(traj.last_id));
      printf("[trajectory_manager] Warning: List of points is full. Last point was removed.\n");
    }

    /* Insert a point before the current one */
    trajectory_decrease_id(&(traj.cur_id));
    traj.points[traj.cur_id] = point;
  }
}

static void trajectory_manage_order_d(struct trajectory_dest *p)
{
  float d_mm_ref = p->starting_d_mm + p->d.mm;
  if (ABS(d_mm_ref - position_get_distance_mm()) < p->d.precision) {
    trajectory_next_point();
  }
  else {
    control_system_set_distance_mm_ref(d_mm_ref);
  }
}

static void trajectory_manage_order_a_abs(struct trajectory_dest *p)
{
  // TODO: Better handle
  if (ABS(p->a.deg - position_get_angle_deg()) < p->a.precision) {
    trajectory_next_point();
  }
  else {
    control_system_set_angle_deg_ref(p->a.deg);
  }
}

static void trajectory_manage_order_a_rel(struct trajectory_dest *p)
{
  float a_deg_ref = p->starting_a_deg + p->a.deg;
  if (ABS(a_deg_ref - position_get_angle_deg()) < p->a.precision) {
    trajectory_next_point();
  }
  else {
    control_system_set_angle_deg_ref(a_deg_ref);
  }
}

static void trajectory_manage_order_pause(struct trajectory_dest *p)
{
  control_system_set_distance_mm_ref(p->starting_d_mm);
  control_system_set_angle_deg_ref(p->starting_a_deg);
}

static inline void trajectory_update()
{
  /* Nothing to do if there is no point in the list */
  if (traj.cur_id == traj.last_id) {
    return;
  }

  /* Get current point reference */
  struct trajectory_dest *p = traj.points + traj.cur_id;

  /* Get current starting position in distance and angle */
  if (!p->is_init) {
    p->starting_d_mm = position_get_distance_mm();
    p->starting_a_deg = position_get_angle_deg();
    p->is_init = 1;
  }

  /* Set new reference according to point type */
  switch (p->type) {
    case D:
      trajectory_manage_order_d(p);
      break;
    case A_ABS:
      trajectory_manage_order_a_abs(p);
      break;
    case A_REL:
      trajectory_manage_order_a_rel(p);
      break;
    case PAUSE:
      trajectory_manage_order_pause(p);
      break;
    default:
      break;
  }
}

// work in progress
static inline void trajectory_update_smoothly()
{
  /* Nothing to do if there is no point in the list */
  if (traj.cur_id == traj.last_id) {
    return;
  }

  /* Get current point reference */
  struct trajectory_dest *p = traj.points + traj.cur_id;

  /* Get current starting position in distance and angle */
  /*if (!p->is_init) {
    p->starting_d_mm = position_get_distance_mm();
    p->starting_a_deg = position_get_angle_deg();
    p->is_init = 1;
  }*/

  //static float target_x = 0;
  //static float target_y = 0;


  /* Set new reference according to point type */
  switch (p->type) {
    case D:
      //float d_mm_ref = p->starting_d_mm + p->d.mm;
      break;
    case A_ABS:
      trajectory_manage_order_a_abs(p);
      break;
    case A_REL:
      trajectory_manage_order_a_rel(p);
      break;
    case PAUSE:
      trajectory_manage_order_pause(p);
      break;
    default:
      break;
  }
}
