#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "position_manager.h"
#include "smooth_traj_manager.h"
#include "cli.h"
#include "platform.h"
#include "astar.h"
#include "servo.h"
#include "actions.h"


extern ausbeeServo servo_clapet;
extern ausbeeServo servo_grip;

void cli_task(void *);

void cli_start()
{
  xTaskCreate(cli_task, (const signed char *)"CLI", 200, NULL, 1, NULL );
}

static int cli_getchar(void)
{
  while (USART_GetFlagStatus(PLATFORM_USART_PRINTF, USART_IT_RXNE) == 0){
	  vTaskDelay(100 / portTICK_RATE_MS);
  }

  int c = USART_ReceiveData(PLATFORM_USART_PRINTF);

  if (c == CLI_END_CHAR) {
    printf("\r\n");
  }
  else if (c == CLI_DEL_CHAR)
  {
	  return c;
  }
  else
  {
	  char s[2] = {c, '\0'};
	  printf("%s", s);
  }

  return c;
}

/*static void cli_getline(char *buff, uint8_t line_length)
{
  int c = 0;
  int i = 0;

  memset(buff, 0, line_length * sizeof(char));

  while (((c = getchar()) != EOF) && ((char)c != CLI_END_CHAR)) {
    if ((char)c == CLI_DEL_CHAR) {
      if (i > 0) {
        buff[--i] = 0;
        printf("\b \b");
      }
    }
    else if (i < line_length-1) {
      buff[i++] = (char)c;
      printf("%c", (char)c);
    }
  }
  printf("\r\n");
}*/

static void cli_getstr(char *buff, uint8_t str_length)
{
  int c = 0;
  int i = 0;

  memset(buff, 0, str_length * sizeof(char));

  while (((c = cli_getchar()) != EOF) && ((char)c == CLI_DELIMITER));

  do {
	  if (c == CLI_DEL_CHAR)
	  {
		  if (i > 0)
		  {
			  buff[--i] = 0;
		  }
	  }
	  else
	  {
		  buff[i++] = (char)c;
	  }

	  if (i > str_length)
		  break;

  } while (((c = cli_getchar()) != EOF) && ((char)c != CLI_DELIMITER) && ((char) c != CLI_END_CHAR));
}

#define FLOAT_LENGTH 20
static float cli_getfloat(void)
{
  char buff[FLOAT_LENGTH] = {0};

  cli_getstr(buff, FLOAT_LENGTH);

  float value = strtof(buff, NULL);

  return value;
}

#define ARG_LENGTH 20
void cli_task(void *data)
{
  (void)data;

  char command = 0;
  float value = 0;
  char arg[ARG_LENGTH] = {0};
  char arg2[ARG_LENGTH] = {0};
  int c = 0;

  for (;;) {
    value = 0;

    //printf("$ ");
    command = cli_getchar();
    if (command == 'd' || command == 'a' || command == 'c' || command == '*') {
      value = cli_getfloat();
    }
    else if (command == 'p') {
      cli_getstr(arg, ARG_LENGTH);
    }
    else if (command == 's') {
      cli_getstr(arg, ARG_LENGTH);
      value = cli_getfloat();
    }
    else if (command == 'm' || command == 'r') {
      cli_getstr(arg, ARG_LENGTH);
      cli_getstr(arg2, ARG_LENGTH);
    }
    else {
      while (((c = cli_getchar()) != EOF) && ((char)c != CLI_END_CHAR));
    }

    if (command == 'd') {
      smooth_traj_goto_d_mm(value);
      printf("Distance: %f\r\n", (double)value);
    }
    else if (command == 'a') {
      smooth_traj_goto_a_deg(value);
      printf("Angle: %f\r\n", (double)value);
    }
    else if (command == 'c') {
      float y = cli_getfloat();
      smooth_traj_goto_xy_mm(value, y);
      printf("Goto xy: %f  %f\r\n", (double)value, (double)y);
    }
    else if (command == '*') {
      float y = cli_getfloat();
      set_startCoor(positionMmToCoordinate(position_get_x_mm(), position_get_y_mm()));
      set_goalCoor(positionMmToCoordinate(value, y));
      printf("Goto xy: %f  %f\r\n", (double)value, (double)y);
      astarMv();
    }
    else if (command == 'z') {
      smooth_traj_end();
      printf("stop mvt\r\n");
	}
    else if (command == 's') {
      if (!strncmp(arg, "speed_high", ARG_LENGTH)) {
        control_system_set_speed_high();
        printf("Max speed.\r\n");
      }
      else if (!strncmp(arg, "speed_medium", ARG_LENGTH)) {
        control_system_set_speed_medium();
        printf("Medium speed.\r\n");
      }
      else if (!strncmp(arg, "speed_low", ARG_LENGTH)) {
        control_system_set_speed_low();
        printf("Low speed.\r\n");
      }
      else if (!strncmp(arg, "speed", ARG_LENGTH)) {
        control_system_set_speed_ratio(value);
        printf("Speed: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "pid_d_p", ARG_LENGTH)) {
        ausbee_pid_set_kp(control_system_get_pid_distance(), value);
        printf("Distance P: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "pid_d_i", ARG_LENGTH)) {
        ausbee_pid_set_ki(control_system_get_pid_distance(), value);
        printf("Distance I: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "pid_d_d", ARG_LENGTH)) {
        ausbee_pid_set_kd(control_system_get_pid_distance(), value);
        printf("Distance D: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "pid_a_p", ARG_LENGTH)) {
        ausbee_pid_set_kp(control_system_get_pid_angle(), value);
        printf("Angle P: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "pid_a_i", ARG_LENGTH)) {
        ausbee_pid_set_ki(control_system_get_pid_angle(), value);
        printf("Angle I: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "pid_a_d", ARG_LENGTH)) {
        ausbee_pid_set_kd(control_system_get_pid_angle(), value);
        printf("Angle D: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "axle_track", ARG_LENGTH)) {
        position_set_axle_track_mm((double)value);
        printf("Axle track: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "speed_d", ARG_LENGTH)) {
       	control_system_set_distance_max_speed((double)value);
        printf("distance max speed: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "speed_a", ARG_LENGTH)) {
    	control_system_set_angle_max_speed((double)value);
        printf("angle max speed: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "acc_d", ARG_LENGTH)) {
        control_system_set_distance_max_acc((double)value);
        printf("distance max acceleration: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "acc_a", ARG_LENGTH)) {
        control_system_set_angle_max_acc((double)value);
        printf("angle max acceleration: %f\r\n", (double)value);
      }
      else {
        printf("Invalid argument '%s'.\r\n", arg);
      }
    }
    else if (command == 'p') {
      if (!strncmp(arg, "x", ARG_LENGTH)) {
        printf("Robot x mm: %f\r\n", (double)position_get_x_mm());
      }
      else if (!strncmp(arg, "y", ARG_LENGTH)) {
        printf("Robot y mm: %f\r\n", (double)position_get_y_mm());
      }
      else if (!strncmp(arg, "a", ARG_LENGTH)) {
        printf("Robot angle deg: %f\r\n", (double)position_get_angle_deg());
      }
      else if (!strncmp(arg, "d", ARG_LENGTH)) {
        printf("Robot distance mm: %f\r\n", (double)position_get_distance_mm());
      }
      else if (!strncmp(arg, "enc_r", ARG_LENGTH)) {
        printf("Right encoder value: %f\r\n", (double)position_get_right_encoder());
      }
      else if (!strncmp(arg, "enc_l", ARG_LENGTH)) {
        printf("Left encoder value: %f\r\n", (double)position_get_left_encoder());
      }
      /*else if (!strncmp(arg, "cur_id", ARG_LENGTH)) {
        printf("Traj manager cur_id: %d\r\n", (int)trajectory_get_cur_id());
      }
      else if (!strncmp(arg, "last_id", ARG_LENGTH)) {
        printf("Traj manager last_id: %d\r\n", (int)trajectory_get_last_id());
      }*/
      else if (!strncmp(arg, "pid", ARG_LENGTH)) {
        printf("Distance PID: %f, %f, %f\r\n", (double)ausbee_pid_get_kp(control_system_get_pid_distance()),
                                               (double)ausbee_pid_get_ki(control_system_get_pid_distance()),
                                               (double)ausbee_pid_get_kd(control_system_get_pid_distance()));
        printf("Angle PID:    %f, %f, %f\r\n", (double)ausbee_pid_get_kp(control_system_get_pid_angle()),
                                               (double)ausbee_pid_get_ki(control_system_get_pid_angle()),
                                               (double)ausbee_pid_get_kd(control_system_get_pid_angle()));
      }
      else if (!strncmp(arg, "graph", ARG_LENGTH)) {
        printGraphe();
      }
      else {
        printf("Invalid argument '%s'.\r\n", arg);
      }
    }
    else if (command == 'm'){
      if (!strncmp(arg, "lift", ARG_LENGTH)) {
        if (!strncmp(arg2, "up", ARG_LENGTH)) {
          action_raise_lift();
          printf("Lift up\r\n");
        }
        else if (!strncmp(arg2, "down", ARG_LENGTH)) {
          action_lower_lift();
          printf("Lift down\r\n");
        }
        else {
          printf("Invalid argument '%s'.\r\n", arg2);
        }
      }
      else if (!strncmp(arg, "grip", ARG_LENGTH)) {
    	  ausbeeSetAngleServo(&servo_grip, atoi(arg2));
    	  printf("command servo.\r\n");
      }
      else if (!strncmp(arg, "clapet", ARG_LENGTH)) {
		  ausbeeSetAngleServo(&servo_clapet, atoi(arg2));
		  printf("command servo.\r\n");
      }
      else {
        printf("Invalid argument '%s'.\r\n", arg);
      }
    }
    else if (command == 'h') {
      printf("Help:\r\n");
      printf("  Available commands are:\r\n");
      printf("  d <float>: Go forward/backward with the specified distance in mm.\r\n");
      printf("  a <float>: Rotate with the specified angle in degrees.\r\n");
      printf("  c <float> <float>: goto to the position (x, y) in mm.\r\n");
      printf("  * <float> <float>: goto to the position (x, y) in mm with A*.\r\n");
      printf("  s <arg> <value>:   Set internal value.\r\n");
      printf("             <value> should be a float.\r\n");
      printf("             <arg> can be one of:\r\n");
      printf("             speed_high:   set highest translation and rotation speed.\r\n");
      printf("             speed_medium: set medium translation and rotation speed.\r\n");
      printf("             speed_low:    set low translation and rotation speed.\r\n");
      printf("             speed :       set translation and rotation speed ratio to value (0 <= value <= 1).\r\n");
      printf("             pid_d_p :     set distance PID proportional value.\r\n");
      printf("             pid_d_i :     set distance PID integral value.\r\n");
      printf("             pid_d_d :     set distance PID derivative value.\r\n");
      printf("             pid_a_p :     set angle PID proportional value.\r\n");
      printf("             pid_a_i :     set angle PID integral value.\r\n");
      printf("             pid_a_d :     set angle PID derivative value.\r\n");
      printf("             axle_track :  set axle track in mm.\r\n");
      printf("             speed_d :     set max speed for distance.\r\n");
      printf("             speed_a :     set max speed for angle.\r\n");
      printf("             acc_d :       set max acceleration for distance.\r\n");
      printf("             acc_a :       set max acceleration for angle.\r\n");
      printf("  m <arg> <arg2> : move an actuator\r\n");
      printf("             <arg> can be one of: \r\n");
      printf("             arm_l: left_arm \r\n");
      printf("             arm_r: right_arm \r\n");
      printf("                <arg2> can be one of: \r\n");
      printf("                close: close the arm \r\n");
      printf("                open: open the arm \r\n");
      printf("  p <arg>:   Print internal value.\r\n");
      printf("             <arg> can be one of:\r\n");
      printf("             x:        print robot's x position.\r\n");
      printf("             y:        print robot's y position.\r\n");
      printf("             a:        print robot's angle.\r\n");
      printf("             d:        print robot's distance.\r\n");
      printf("             enc_l:    print left encoder's value.\r\n");
      printf("             enc_r:    print right encoder's value.\r\n");
      printf("             cur_id:   print Traj manager's current point id.\r\n");
      printf("             last_id:  print Traj manager's last point id.\r\n");
      printf("             pid:      print PID.\r\n");
      printf("             graphe:   print A* graphe.\r\n");
    }
    else {
      printf("Unknown command '%c'. Type 'h' for help.\r\n", command);
    }

    vTaskDelay(100 / portTICK_RATE_MS);
  }
}
