#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "control_system_debug.h"
#include "position_manager.h"

#define PI 3.1415926535

extern struct control_system am;

void control_system_debug_task(void *);

void control_system_debug_start()
{
  xTaskCreate(control_system_debug_task, (const signed char *)"ControlSystemDebug", 200, NULL, 1, NULL );
}

void control_system_debug_task(void *data)
{
  (void) data;

  for (;;) {
    //printf("Right Measure:          %f: 1;"    , (double)ausbee_cs_get_measure(&(am.csm_right_motor)));
    //printf("Right Filtered measure: %f: 10;"   , (double)ausbee_cs_get_filtered_measure(&(am.csm_right_motor)));
    //printf("Right Reference:        %f: 1;"    , (double)ausbee_cs_get_reference(&(am.csm_right_motor)));
    //printf("Right Error:            %f: 1;"    , (double)ausbee_cs_get_error(&(am.csm_right_motor)));
    //printf("Right Command:          %f: 10\r\n", (double)ausbee_cs_get_command(&(am.csm_right_motor)));

    //printf("Left Measure:          %f: 1;"    , (double)ausbee_cs_get_measure(&(am.csm_left_motor)));
    //printf("Left Filtered measure: %f: 10;"   , (double)ausbee_cs_get_filtered_measure(&(am.csm_left_motor)));
    //printf("Left Reference:        %f: 1;"    , (double)ausbee_cs_get_reference(&(am.csm_left_motor)));
    //printf("Left Error:            %f: 1;"    , (double)ausbee_cs_get_error(&(am.csm_left_motor)));
    //printf("Left Command:          %f: 10\r\n", (double)ausbee_cs_get_command(&(am.csm_left_motor)));

    printf("Distance reference mm: %f: 1;"   , (double)ausbee_cs_get_reference(&(am.csm_distance)));
    //printf("Distance filt ref mm:  %f: 1;"   , (double)ausbee_cs_get_filtered_reference(&(am.csm_distance)));
    printf("Distance measure mm:   %f: 1;"   , (double)ausbee_cs_get_measure(&(am.csm_distance)));
    printf("Distance error mm:     %f: 1;"   , (double)ausbee_cs_get_error(&(am.csm_distance)));
    //printf("Distance error sum mm: %f: 1;"   , (double)ausbee_pid_get_error_sum(&(am.pid_distance)));
    //printf("Distance error diff mm: %f: 1;"   , (double)ausbee_pid_get_error_diff(&(am.pid_distance)));
    printf("Distance command mm:   %f: 10\r\n", (double)ausbee_cs_get_command(&(am.csm_distance)));

    printf("Angle reference deg:   %f: 1;"   , (double)(180.0 / PI * ausbee_cs_get_reference(&(am.csm_angle))));
    //printf("Angle filt ref deg:    %f: 1;"   , (double)(180.0 / PI * ausbee_cs_get_filtered_reference(&(am.csm_angle))));
    printf("Angle measure deg:     %f: 1;"   , (double)(180.0 / PI * ausbee_cs_get_measure(&(am.csm_angle))));
    printf("Angle error deg:       %f: 1;"   , (double)(180.0 / PI * ausbee_cs_get_error(&(am.csm_angle))));
    printf("Angle command deg:     %f: 1\r\n", (double)(180.0 / PI * ausbee_cs_get_command(&(am.csm_angle))));

    //printf("Robot x mm: %f;   y mm: %f      ", (double)position_get_x_mm(), (double)position_get_y_mm());
    //printf("EncG: %d   EncD: %d\r\n", (int)position_get_left_encoder(), (int)position_get_right_encoder());

    vTaskDelay(CONTROL_SYSTEM_PERIOD_S * 1000 / portTICK_RATE_MS);
  }
}
