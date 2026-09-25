#ifndef DISPLAY_H
#define DISPLAY_H


#include "main.h"

//Declares an integer and functions.
extern int selected_profile;
void initialization_display();
void run_selected_auto(void);
void run_selected_GIF(void);
void pid_screen_display(float& kP, float& kI, float& kD);
void pid_constant_updating();
extern int selected_program;
extern int auto_type;
extern int selected_debug_option;

typedef enum autonomous_type {
  AUTONOMOUS_RED = 0,
  AUTONOMOUS_BLUE = 1,
  AUTONOMOUS_SKILLS = 2
} autonomous_type_e_t;

#endif

