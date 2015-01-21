#include <stdio.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "position_manager.h"
#include "smooth_traj_manager.h"

/*enum smooth_traj_order_type {
  PAUSE, D, A_ABS, A_REL
};*/

enum smooth_traj_when {
  NOW, END
};

struct smooth_traj_dest {
 
  float x;
  float y;


  //enum smooth_traj_order_type type;
};


/******************** Global variable ********************/
struct smooth_traj_manager {
  struct smooth_traj_dest points[SMOOTH_TRAJ_MAX_NB_POINTS];
  uint32_t cur_id;
  uint32_t last_id;
  float previous_waypoint_dist;
} traj;


#define ABS(x) (((x) < 0)? -(x): (x))
#define SQUARE(x) ((x)*(x))
#define DISTANCE(x1, y1, x2, y2) (sqrtf(SQUARE((x1-x2)) + SQUARE((y1-y2))))

/********************   Prototypes   ********************/
void smooth_traj_task(void *data);
static inline void smooth_traj_update();
static void smooth_traj_add_point(struct smooth_traj_dest point, enum smooth_traj_when when);


/******************** User functions ********************/

void smooth_traj_init()
{
  traj.cur_id = 0;
  traj.last_id = 0;
  traj.previous_waypoint_dist = 1.E20f;
}

void smooth_traj_start()
{
  xTaskCreate(smooth_traj_task, (const signed char *)"TrajectoryManager", 200, NULL, 1, NULL );
}

void smooth_traj_end()
{
  traj.cur_id = traj.last_id;
}

int smooth_traj_is_ended()
{
  return (traj.cur_id == traj.last_id);
}

void smooth_traj_next_point()
{
  /* Update list pointer if not empty */
  if (!smooth_traj_is_ended()) {
    traj.cur_id = (traj.cur_id+1) % SMOOTH_TRAJ_MAX_NB_POINTS;
  }
}

uint32_t smooth_traj_get_cur_id()
{
  return traj.cur_id;
}

uint32_t smooth_traj_get_last_id()
{
  return traj.last_id;
}

/******************** Movement functions ********************/

static int smooth_traj_is_paused()
{
	//TODO
	return 0;
}

void smooth_traj_pause()
{
  if (!smooth_traj_is_paused()) {
      /*TODOstruct smooth_traj_dest dest;
      dest.type = PAUSE;
      smooth_traj_add_point(dest, NOW);*/
  }

  /* Force update now to stop more quickly */
  smooth_traj_update(traj);
}

void smooth_traj_resume()
{
  while (smooth_traj_is_paused()) {
    smooth_traj_next_point();
  }
}

void smooth_traj_goto_xy_mm(float x, float y)
{
	struct smooth_traj_dest dest;
	
	dest.x = x;
	dest.y = y;
	
	smooth_traj_add_point(dest, END);
}

/****************** Internal functions ******************/

void smooth_traj_task(void *data)
{
  (void)data;

  for (;;) {
    smooth_traj_update();

    vTaskDelay(SMOOTH_TRAJ_UPDATE_PERIOD_S * 1000 / portTICK_RATE_MS);
  }
}

static inline int smooth_traj_is_full()
{
  return (((traj.last_id+1) % SMOOTH_TRAJ_MAX_NB_POINTS) == traj.cur_id);
}

static inline void smooth_traj_decrease_id(uint32_t *id)
{
  *id = (*id + SMOOTH_TRAJ_MAX_NB_POINTS - 1) % SMOOTH_TRAJ_MAX_NB_POINTS;
}

void smooth_traj_add_point(struct smooth_traj_dest point, enum smooth_traj_when when)
{

  if (when == END) {
    if (smooth_traj_is_full()) {
      printf("[smooth_traj_manager] Warning: List of points is full. Last point not added.\n");
      return;
    }

    /* New points are added at the end of the list */
    traj.points[traj.last_id] = point;

    /* Update end of list pointer */
    traj.last_id = (traj.last_id + 1) % SMOOTH_TRAJ_MAX_NB_POINTS;
  }
  else if (when == NOW) {
    if (smooth_traj_is_full()) {
      smooth_traj_decrease_id(&(traj.last_id));
      printf("[smooth_traj_manager] Warning: List of points is full. Last point was removed.\n");
    }

    /* Insert a point before the current one */
    smooth_traj_decrease_id(&(traj.cur_id));
    traj.points[traj.cur_id] = point;
  }
}


/*static inline void smooth_traj_update()
{
  // Nothing to do if there is no point in the list 
  if (traj.cur_id == traj.last_id) {
    return;
  }

  // Get current point reference
  struct smooth_traj_dest *p = traj.points + traj.cur_id;

  // Get current starting position in distance and angle
  if (!p->is_init) {
    p->starting_d_mm = position_get_distance_mm();
    p->starting_a_deg = position_get_angle_deg();
    p->is_init = 1;
  }

  // Set new reference according to point type
  switch (p->type) {
    case D:
      smooth_traj_manage_order_d(p);
      break;
    case A_ABS:
      smooth_traj_manage_order_a_abs(p);
      break;
    case A_REL:
      smooth_traj_manage_order_a_rel(p);
      break;
    case PAUSE:
      smooth_traj_manage_order_pause(p);
      break;
    default:
      break;
  }
}*/

// work in progress
static inline void smooth_traj_update()
{
	/* Nothing to do if there is no point in the list */
	if (traj.cur_id == traj.last_id) {
		return;
	}

	// reference to the next waypoint
	struct smooth_traj_dest* next1 = traj.points + traj.cur_id;
	float next1_dist = DISTANCE(position_get_x_mm(), position_get_y_mm(), next1->x, next1->y);
	// position the robot want to reach
	float target_x = 0, target_y = 0;


	uint32_t next_id = (traj.cur_id+1) % SMOOTH_TRAJ_MAX_NB_POINTS;

	// at least two waypoints
	if (0)//next_id != traj.last_id)
	{
		// find the second waypoint
		struct smooth_traj_dest* next2 = traj.points + next_id;

		float d = SMOOTH_TRAJ_STEER_DISTANCE_MM;
		d -= next1_dist;

		if (d > 0.f)
		{
			float direction_x = next2->x - next1->x;
			float direction_y = next2->y - next1->y;
			float direction_length = sqrt(SQUARE(direction_x) + SQUARE(direction_y));
			direction_x = direction_x / direction_length * d;
			direction_y = direction_y / direction_length * d;

			target_x = next1->x + direction_x;
			target_y = next1->y + direction_y;
		}
		else // the next waypoint is too far, so it's our target
		{
			target_x = next1->x;
			target_y = next1->y;
		}

		// if the robot go away the current waypoint, we switch to the next
		if (next1_dist > traj.previous_waypoint_dist)
		{
			smooth_traj_next_point();
		}
		traj.previous_waypoint_dist = next1_dist;

	}
	// only one more waypoint
	else
	{
		target_x = next1->x;
		target_y = next1->y;

		// Waypoint reachs
		if (next1_dist < SMOOTH_TRAJ_DEFAULT_PRECISION_D_MM)
		{
			smooth_traj_next_point();
		}
	}


	// Compute the angle and distance to send to the control system
	float angle_ref = atan2f(-(target_x - position_get_x_mm()), target_y - position_get_y_mm());
	float remaining_dist = DISTANCE(position_get_x_mm(), position_get_y_mm(), target_x, target_y);

	printf("tar x:%d  y:%d  dist:%f  a:%f\r\n", (int)target_x, (int)target_y, (double)next1_dist, (double)angle_ref);

	control_system_set_distance_mm_ref(position_get_distance_mm() + remaining_dist);
	control_system_set_angle_rad_ref(angle_ref);
}