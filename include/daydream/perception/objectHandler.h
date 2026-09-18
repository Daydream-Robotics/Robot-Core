#ifndef OBJECT_HANDLER
#define OBJECT_HANDLER

#include <array>
#include <optional>
#include <cstdint>
#include "pros/rtos.hpp"

// disconnect time ms
constexpr uint32_t DISCONNECT_TIMEOUT_MS = 500;

// sets type to 1 byte
enum class GamePiece : uint8_t {
    RED_BALL,
    BLUE_BALL,
    LONG_GOAL,
    MID_GOAL_1,
    MID_GOAL_2,
    MID_GOAL_3,
    MID_GOAL_4,
    COUNT
};

// gets the index of an enum 
constexpr size_t toIndex(GamePiece g) {
    return static_cast<size_t>(g);
}

// each object will take this type
struct GamePieceData{
	double x{};
	double y{};
	double conf{};
};

// array that can hold GamePieceData types or nothing of max size of the enum
using GamePieceArray = std::array<std::optional<GamePieceData>, toIndex(GamePiece::COUNT)>;
extern bool frame_ready;
    
// scary multithreading
extern pros::Mutex frame_mutex; // this is for access safety
void UpdateFrame_task_fn(void* param);

// public functions
const GamePieceArray GetFrame();
std::optional<GamePieceData> GetObject(GamePiece gamePiece);
bool IsConnected();
#endif