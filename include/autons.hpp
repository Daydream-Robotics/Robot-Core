#ifndef AUTOPROGRAMS_H
#define AUTOPROGRAMS_H

//Declares the autonomous and skills functions f

extern volatile bool auton_complete;
extern bool filter_check;
void kill_filter(void);

void red_solo_awp(void);
void blue_solo_awp(void);

void left_red_one_auton(void);
void left_red_two_auton(void);
void left_red_three_auton(void);

void left_blue_one_auton(void);
void left_blue_two_auton(void);
void left_blue_three_auton(void);

void right_blue_one_auton(void);
void right_blue_two_auton(void);
void right_blue_three_auton(void);

void right_red_one_auton(void);
void right_red_two_auton(void);
void right_red_three_auton(void);

void left_red_elim_match_auton(void);
void right_red_elim_match_auton(void);
void left_blue_elim_match_auton(void);
void right_blue_elim_match_auton(void);

void programming_skills(void);
void programming_skills_middle(void);
void driver_skills(void);
void test_auton(void);
void disabled_auton(void);
void disabled_ladder_auton(void);

typedef enum filter_color {
  E_RED = 0,
  E_BLUE = 1,
} filter_color_e_t;

#endif
