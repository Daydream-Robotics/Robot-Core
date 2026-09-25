#include "autons.hpp"
#include "display.hpp"
#include "api.h"
volatile bool auton_complete = false;


// ! template
void solo_awp(int auton_color) {
    const int64_t start = pros::millis();
    auto_type = auton_color;
    //do auto stuff
    
    printf("done: %f\n", ((double)(pros::millis() - start))/1000);
}

// defines function for unicolor middle goal auton
void middle_goal(int auton_color) {
    //gutted
}

void left_seven_ball(int auton_color) {
    //gutted
}

void low_goal(int auton_color) {
    //gutted
}

void right_seven_ball(int auton_color) {
    //gutted
}

void test_auton() {
    const int64_t start = pros::millis();
    int64_t filter_time;
    auto_type = AUTONOMOUS_RED;
    
}

void disabled_auton() {
    const int64_t start = pros::millis();
    int64_t filter_time;
    auto_type = AUTONOMOUS_RED;
    
    printf("done: %f\n", ((double)(pros::millis() - start))/1000);
}

void disabled_ladder_auton() {
    const int64_t start = pros::millis();
    int64_t filter_time;
    auto_type = AUTONOMOUS_RED;
    
    printf("done: %f\n", ((double)(pros::millis() - start))/1000);
}

void red_solo_awp() {
    solo_awp(AUTONOMOUS_RED);
}

void left_red_elim_match_auton() {
    const int64_t start = pros::millis();
    int64_t filter_time;
    auto_type = AUTONOMOUS_RED;
    printf("done: %f\n", ((double)(pros::millis() - start))/1000);
}

void right_red_elim_match_auton() {
    const int64_t start = pros::millis();
    int64_t filter_time;
    auto_type = AUTONOMOUS_RED;
    printf("done: %f\n", ((double)(pros::millis() - start))/1000);
}

void left_red_one_auton() {
    middle_goal(AUTONOMOUS_RED);
}

void left_red_two_auton() {
    left_seven_ball(AUTONOMOUS_RED);
}

void left_red_three_auton() {
    const int64_t start = pros::millis();
    int64_t filter_time;
    auto_type = AUTONOMOUS_RED;
    printf("done: %f\n", ((double)(pros::millis() - start))/1000);
}

void right_red_one_auton() {
    low_goal(AUTONOMOUS_RED);
}

void right_red_two_auton() {
    right_seven_ball(AUTONOMOUS_RED);
}

void right_red_three_auton() {
    const int64_t start = pros::millis();
    int64_t filter_time;
    auto_type = AUTONOMOUS_RED;
    printf("done: %f\n", ((double)(pros::millis() - start))/1000);
}


void blue_solo_awp() {
    solo_awp(AUTONOMOUS_BLUE);
}

void left_blue_elim_match_auton() {
    const int64_t start = pros::millis();
    int64_t filter_time;
    auto_type = AUTONOMOUS_BLUE;
    printf("done: %f\n", ((double)(pros::millis() - start))/1000);
}

void right_blue_elim_match_auton() {
    const int64_t start = pros::millis();
    int64_t filter_time;
    auto_type = AUTONOMOUS_BLUE;
    printf("done: %f\n", ((double)(pros::millis() - start))/1000);
}


void left_blue_one_auton() {
    middle_goal(AUTONOMOUS_BLUE);
}

void left_blue_two_auton() {
    left_seven_ball(AUTONOMOUS_BLUE);
}

void left_blue_three_auton() {
    const int64_t start = pros::millis();
    int64_t filter_time;
    auto_type = AUTONOMOUS_BLUE;
    printf("done: %f\n", ((double)(pros::millis() - start))/1000);
}


void right_blue_one_auton() {
    low_goal(AUTONOMOUS_BLUE);
}

void right_blue_two_auton() {
    right_seven_ball(AUTONOMOUS_BLUE);
}

void right_blue_three_auton() {
    const int64_t start = pros::millis();
    int64_t filter_time;
    auto_type = AUTONOMOUS_BLUE;
    printf("done: %f\n", ((double)(pros::millis() - start))/1000);
}


void programming_skills() {
     // prog skills
}

void programming_skills_middle() {

}

void driver_skills() {
    const int64_t start = pros::millis();
    int64_t filter_time;
    auto_type = AUTONOMOUS_SKILLS;
   
    
    printf("done: %f\n", ((double)(pros::millis() - start))/1000);
}
