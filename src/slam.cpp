#include "main.h"
#include "slam.h"
#include "objectHandler.h"
#include "constants.h"
#include "subsystems.hpp"
#include "autonomous.hpp"



// #include <cmath>
// #include <string>

// we will set these ports in main.h
// pros::MotorGroup left_mg({1, -2, 3});
// pros::MotorGroup right_mg({-4, 5, -6});

// pros::MotorGroup allDrive({1, -2, 3, -4, 5, -6});

// pros::IMU jerry({11});

// move?
void move(int x)
{
    // k
    while (true)
    {
        // movePID

        // check for flag 4a
        // collect
        // return to path

        // chack for flag 4b

        // check for flag 4c
    }
}


// ##########################
// #   FAVORITE NEW WORDS   #
// # ---------------------- #
// #  acompleihsed          #
// #  previus               #
// #  rightj con            #
// #  robo                  #
// #  turniung              #
// #  Equa                  #
// #  pixles                #
// #  intergral             #
// ##########################

// collects a ball which color is designated by the parameter gamePiece
// the parameter can be assigned to RED_BALL or BLUE_BALL (if for whatever reason you can do other game objects)
void collect(GamePiece gamePiece, int isLoading)
{
    using namespace std::chrono;   
    Autonomous auton = Autonomous();
    int count = 0;


    //  setting the weight for how fast the move veloicty will be
    float vel_weight = 255;
    int humpCount = 0;
    pros::lcd::print(1, "test");
    // declares a GamePieceData variable which is assigned to getObject()
    printf("[Collect] Calling findBall...\n");
    std::optional<GamePieceData> ball =  findBall(gamePiece);
    printf("[Collect] findBall returned.\n");
    
    double error, integral=0, derivative=0, pid_total;
    std::optional<double> previousError;
    int turnSpeed;
    int empty_frames = 0;

    int x = 0;
    // needs alternate exit
    while (not frame_ready and IsConnected()) {
        pros::lcd::print(2,"%d", x);
        pros::delay(10);
        x++;
        continue;
    }
    // pros::lcd::print(1,"connected");
     move_intake(STOP, HIGH_VOLTAGE, HIGH_VOLTAGE);


     // TODO: add alternate exits to loop
    std::optional<int> old_x, old_y;
    int times_far_from_old = 0;

    do {
        /////-------------------------////
        // commented this out and it worked 
        // Do not remove
        //
        // if (not IsConnected()) {
        //     pros::lcd::print(1,"not connected");
        //     break;
        // }
        /////-------------------------////
        if(not frame_ready) {
            pros::delay(10);
            // pros::lcd::print(1,"frame not ready");
            continue;
        }
        // pros::lcd::print(1,"frame ready");

        ball = GetObject(gamePiece);

        if (ball.has_value()) {
            // check if current ball is near the location of previous detected ball
            if (old_x.has_value() && old_y.has_value()) {
                if (not checkIfLocationClose(0.05, old_x.value(), old_y.value(), ball->x, ball->y)) {
                    times_far_from_old++;
                } else {
                    times_far_from_old = 0;
                }
            }

            // exit if detected ball is too far from previously detected ball ten times in a row
            if (times_far_from_old > 10) {
                break;
            }

            error = ball->x - 0.5;                 // makes the relative angle
            integral += error;                              // adds the error to the sum
            if (previousError.has_value()) {
                derivative = error - previousError.value(); // calc derivitve
            }
            empty_frames = 0;

            old_x = ball->x;
            old_y = ball->y;
            
        } else {
            if(isLoading == 4 && empty_frames > 2){
                return;
            }
            empty_frames++;
        }

        pid_total = ((collect_turn_kP * error) + (collect_turn_kI * integral) + (collect_turn_kD * derivative));  // PID Equation
        // turnSpeed = pid_total * MAX_VELOCITY;
        turnSpeed = pid_total * COLLECT_TURN_ADJUSTMENT;
        turnSpeed = std::clamp(std::abs(turnSpeed), COLLECT_MIN_VELOCITY, COLLECT_MAX_VELOCITY);  // Limits the voltage within a range
        turnSpeed = (int)copysign(turnSpeed, pid_total);
        
        // DEBUG
        bool condition = std::abs(error) > MAX_PIXEL_OFFSET;
        printf("[TurnTo] X: %.2f | Error: %.2f | TurnSpeed: %d | Loop Cond: %d\n", 
               ball->x, error, turnSpeed, condition);
        // END DEBUG

        // pros::lcd::print(1, "turning with %d velocity", turnSpeed);

        // //runns if we are matchloading
        // if(isLoading == 1){
        //     pros::lcd::print(5, "is loading = %d", isLoading);
        //     odom.updatePose();
        //     if(abs( prevX - odom.pos_x) < 1){
        //         isLoading++;
        //         pros::lcd::print(3, "Entered Collect pt2");
        //         collect(gamePiece, isLoading);
        //         return;
        //     }
        //     prevX = auton.pos_x;
        // }

        

        
        if(isLoading == 3){
            if(empty_frames > 2 || humpCount > 4 && humpCount >1)return;

            // pros::lcd::print(5, "is loading = %d", isLoading);
            leftMotors.move_velocity(-20);
            rightMotors.move_velocity(-20);
            pros::delay(400);
            leftMotors.move_velocity(50);
            rightMotors.move_velocity(50);            
            pros::delay(500);
            humpCount++;
            
        }
        else if(isLoading == 2){
            if(gamePiece == GamePiece::BLUE_BALL)gamePiece = GamePiece::RED_BALL;
            else gamePiece == GamePiece::BLUE_BALL;
            // pros::lcd::print(0,"loading = 2");
            isLoading = 3;   
        }
        else if(isLoading == 1 && std::abs(error) < .01){
        leftMotors.move_velocity(50);
        rightMotors.move_velocity(50);
        }

        else{
        leftMotors.move_velocity(turnSpeed + 50);
        rightMotors.move_velocity(-turnSpeed + 50);
        }



        previousError = error; // resetting the previous Angle to use in the next iteration

        // pros::lcd::print(2, "Error: %lf", error);
        // pros::lcd::print(1, "run count %d", count);



        if(isLoading == 1 && count >= 15)break;
        count++;
        //previus empty frames: 10
    } while (empty_frames < 5);
    
    if(isLoading == 1){
        collect(gamePiece, 2);
    }
    else if(isLoading == 4){
        leftMotors.move_velocity(-100);
        rightMotors.move_velocity(-100);
        pros::delay(400);
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
        
        collect(gamePiece);
        leftMotors.move_velocity(-100);
        rightMotors.move_velocity(-100);
        pros::delay(300);
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
    }
    // DEBUG
    printf("[TurnTo] Exiting loop. Final Error: %.2f\n", error); // DEBUG
    // END DEBUG

    // frontIntake.move_velocity(0);
    // midIntake.move_velocity(0);

    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    // pros::lcd::print(5, "robot stopped turning");
    return;
}

// std::optional<GamePieceData> findBall(GamePiece gamePiece)
// {
//     std::optional<GamePieceData> ball;

//     // DEBUG
//     printf("[FindBall] Searching...\n");
//     // END DEBUG
//     pros::lcd::print(1, "[findBall] at while loop");

//     while (not ball.has_value() && IsConnected())
//     {
//         pros::lcd::print(1, "[findBall] in while loop");
//         if(not frame_ready) {
//             pros::lcd::print(1, "[findBall] not frame ready");
//             pros::delay(10);
//             continue;
//         }
//         // getting the ball
//         ball = GetObject(gamePiece);
        
//         if (ball.has_value()) {
//             printf("[FindBall] Found ball at X: %.2f\n", ball->x);
//             break;
//         }
        
//         pros::lcd::print(1,"turniung right (cant find ball)");
//         leftMotors.move_velocity(20);
//         rightMotors.move_velocity(-20);
        
//         pros::delay(10);
//     }
//     leftMotors.move_velocity(0);
//     rightMotors.move_velocity(0);
//     return ball;
// }

std::optional<GamePieceData> findBall(GamePiece gamePiece)
{
    Autonomous auton = Autonomous();
    std::optional<GamePieceData> ball;

    // DEBUG
    printf("[FindBall] Searching...\n");
    // END DEBUG

    bool searched_right = false;

    double original_angle = odom.getYaw();

    while (not ball.has_value() and IsConnected())
    {
        if(not frame_ready) {
            pros::delay(10);
            continue;
        }
        // getting the ball
        ball = GetObject(gamePiece);
        
        if (ball.has_value()) {
            printf("[FindBall] Found ball at X: %.2f\n", ball->x);
            break;
        }

        double angleDiff = calcAngleDiff(original_angle, odom.getYaw());
        
        // Will turn right 30*, and then left 60* until it finds a ball, if it finds nothing, it will return a sentinel value and we will continue regular intructions
        pros::lcd::print(1,"Searching for Ball");
        if (!searched_right) { // look right for 30*
            leftMotors.move_velocity(20);
            rightMotors.move_velocity(-20);

            if (angleDiff >= 10.0) searched_right = true;
        } else { //look left for 30* from original angle
            leftMotors.move_velocity(-20);
            rightMotors.move_velocity(20);

            if (angleDiff <= -10.0) {
                break;
            }
        }
        
        // Calculate difference in angle between current angle and original heading before the search
        // double angleDiff = calcAngleDiff(original_angle, get_yaw_quaternion());

        // if (angleDiff >= 30.0) { // Search 30* right
        //     searched_right = 1;
        // } else if (angleDiff <= -30.0) { // Search 30* left
        //     leftMotors.move_velocity(0);
        //     rightMotors.move_velocity(0);
        //     pros::lcd::print(1, "Ball not found");
        //     return ball;
        // }

        pros::delay(10);
    }
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    return ball;
}

//
void matchload(bool isFar)
{
    Autonomous auton = Autonomous();
    GamePiece gamePiece;


    if(!isFar)gamePiece = GamePiece::RED_BALL;
    else gamePiece = GamePiece::BLUE_BALL;

    matchloader.set_value(true);

    move_intake(STOP, HIGH_VOLTAGE, HIGH_VOLTAGE);
    // pros::lcd::print(1,"moving intake");
    pros::delay(300);
    collect(gamePiece, 1);
    leftMotors.move_velocity(-100);
    rightMotors.move_velocity(-100);
    pros::delay(200);
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
}


//
void scoreMid()
{
}

//
void scoreHigh()
{
}

void trackingMode(GamePiece GamePiece) {

    bool running = true;
    double error, integral=0, derivative=0, pid_total;
    std::optional<double> previousError;
    int turnSpeed = 0;
    std::optional<GamePieceData> ball;


        pros::lcd::print(1,"program running");
    int n = 1;

    while (not frame_ready) {
        
        pros::delay(10);
        pros::lcd::clear_line(1);
        pros::lcd::print(1,"%d program running", n);
        n++;
        continue;
    }

    pros::lcd::print(1,"program running");

    while (running) {
        if(not frame_ready){
            pros::delay(10);
            continue;
        }
        ball = GetObject(GamePiece);
        error = ball->x - 0.5;
        integral += error;
        if (previousError.has_value()) {
            derivative = error - previousError.value(); // calc derivitve
        }


        pid_total = ((collect_turn_kP * error) + (track_turn_kI * integral) + (track_turn_kD * derivative));  // PID Equation
        // turnSpeed = pid_total * MAX_VELOCITY;
        turnSpeed = pid_total * TRACK_TURN_ADJUSTMENT;
        turnSpeed = std::clamp(std::abs(turnSpeed), TRACK_MIN_VELOCITY, TRACK_MAX_VELOCITY);  // Limits the voltage within a range
        turnSpeed = (int)copysign(turnSpeed, pid_total);
   
        // DEBUG
        bool condition = std::abs(error) > MAX_PIXEL_OFFSET;
        printf("[TurnTo] X: %.2f | Error: %.2f | TurnSpeed: %d | Loop Cond: %d\n", 
               ball->x, error, turnSpeed, condition);
        // END DEBUG

        //pros::lcd::print(1, "turning with %d velocity", turnSpeed);

        leftMotors.move_velocity(turnSpeed);
        rightMotors.move_velocity(-turnSpeed);


        previousError = error; // resetting the previous Angle to use in the next iteration
        if (controller.get_digital_new_press(DIGITAL_LEFT)) {
            break;
        }
        pros::delay(10);
    }

    pros::lcd::print(1, "End Tracking Mode");
    return;
}


double calcAngleDiff(double a, double b){
    double diff = b - a;
    while(diff > 180) diff -= 360;
    while(diff < -180) diff += 360;
    return diff;
}


bool checkIfLocationClose(double percent_differnt, int x1, int y1, int x2, int y2) {
    int diff_x = x1 * percent_differnt;
    int diff_y = y1 * percent_differnt;

    int x_min = x1 - diff_x;
    int x_max = x1 + diff_x;
    int y_min = y1 - diff_y;
    int y_max = y1 + diff_y;

    return x2 >= x_min && x2 <= x_max && y2 >= y_min && y2 <= y_max;
}