#ifndef SLAM_H
#define SLAM_H

#include "objectHandler.h"

constexpr char TEAM = '\0';
// constexpr int FRAME_WIDTH = 256;
// const int FRAME_CENTER = FRAME_WIDTH / 2;
constexpr double FRAME_CENTER = 0.5;

constexpr double COLLECT_TURN_ADJUSTMENT = 1;
constexpr double collect_turn_kP = 10; // 12
constexpr double collect_turn_kI = 0.5;
constexpr double collect_turn_kD = 0;

constexpr double collect_move_kP = 0.05;
constexpr double collect_move_kI = 0;
constexpr double collect_move_kD = 0;

constexpr int COLLECT_MAX_VELOCITY = 255;
constexpr int COLLECT_MIN_VELOCITY = 2;

constexpr int MAX_STRIKES = 5;
constexpr double MAX_DISPLACEMENT = -1;
constexpr double MAX_PIXEL_OFFSET = 0.1;

constexpr double TRACK_TURN_ADJUSTMENT = 1;

constexpr double track_turn_kP = 15; // 12
constexpr double track_turn_kI = 0.5;
constexpr double track_turn_kD = 0;

constexpr int TRACK_MAX_VELOCITY = 255;
constexpr int TRACK_MIN_VELOCITY = 2;

//move?
void move(int x);

// collect ball
void collect(GamePiece gamePiece);
std::optional<GamePieceData> findBall(GamePiece gamePiece);
void turnToGamePiece(GamePiece gamePiece);

//
void matchload(bool isFar = true);

//
void scoreMid(void);

//
void scoreHigh(void);

//

void trackingMode(GamePiece gameBall);

double calcAngleDiff(double a, double b);
bool checkIfLocationClose(double percent_differnt, int x1, int y1, int x2, int y2);

#endif