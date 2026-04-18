#include "objectHandler.h"
#include <cstring>
#include <iostream>
#include "main.h"

pros::Mutex frame_mutex;
bool frame_ready = false;
static uint32_t last_update_time = 0;

// stores the current frame globally
static GamePieceArray g_frame{};

// parses an object for its classid, location, and confidence and stores it in data
static bool parseObject(char*& token, int& classId, GamePieceData& data) {
    // parse class id of the object 
    if (!token || sscanf(token, "%d", &classId) != 1) 
        return false;

    // parse x coordinate of object
    token = strtok(nullptr, ",|\n");
    if (!token || sscanf(token, "%lf", &data.x) != 1) 
        return false;

    // parse y coordinate of object
    token = strtok(nullptr, ",|\n");
    if (!token || sscanf(token, "%lf", &data.y) != 1)
        return false;

    // parse confidence of object
    token = strtok(nullptr, ",|\n");
    if (!token || sscanf(token, "%lf", &data.conf) != 1) 
        return false;

    //moves to next object
    token = strtok(nullptr, ",|\n");

    return true;
}

// to start this task run the line below in initizlie in main
// pros::Task frame_task(UpdateFrameTask, (void*)"PROS_Task_Param", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Vision Frame Update");
void UpdateFrame_task_fn(void* param){
    // pros::lcd::set_text(1, "Started Vision Frame Update Task!");
    int counter = 0;
    while(true) {
        GamePieceArray frame{};
        //size can contain 1 frame of all detected objects
        char inputbuffer[256];

        // pros::lcd::print(7, "%d", counter++);
        
        //begin serial handshake
        printf("A\n");  // !IMPORTANT! Do NOT remove, Keeps AI alive
        fflush(stdout);

        // pros::lcd::set_text(3, "waiting for input");       
        if(!fgets(inputbuffer,sizeof(inputbuffer),stdin)){
            // pros::lcd::set_text(4, "no input");
            pros::delay(10);
            continue;
        }
        // pros::lcd::set_text(5, "input recieved");

        char* token = strtok(inputbuffer,",|\n");
        while(token) {
            int classId;
            GamePieceData data;
            int internalIndex = -1;
    
            if(!parseObject(token, classId, data)) 
                break;
            
            if (classId == 3) {
                bool foundSlot = false;
    
                for(int i = 3; i <=6; i++) {
                    if(!frame[i].has_value()){
                        internalIndex = i;
                        break;
                    }
                }
            }else if(classId >= 0 && classId <= 2) {
                internalIndex = classId;
            }
    
            if(internalIndex != -1) {
                auto& slot = frame[internalIndex];
                if(!slot || data.conf > slot->conf)
                    slot = data;
            }
        }

        // pros::lcd::set_text(4, "Number of objects:" + std::to_string(frame.size()));
        // --- NEW: Debug Printing Logic (Around Line 90) ---
        
        // 1. Get RED_BALL (Index 0)
        // if (frame[0].has_value()) {
        //     // Format: "Red: X=0.52 Y=0.33"
        //     pros::lcd::print(4, "Red: X=%.2f Y=%.2f", frame[0]->x, frame[0]->y);
        // } else {
        //     pros::lcd::set_text(4, "Red: Not Found");
        // }

        // // 2. Get BLUE_BALL (Index 1)
        // if (frame[1].has_value()) {
        //     // Format: "Blue: X=0.88 Y=0.12"
        //     pros::lcd::print(5, "Blue: X=%.2f Y=%.2f", frame[1]->x, frame[1]->y);
        // } else {
        //     pros::lcd::set_text(5, "Blue: Not Found");
        // }
    
        frame_mutex.take();
        g_frame = frame;
        frame_ready = true;
        last_update_time = pros::millis();
        frame_mutex.give();

        // this is so this task gives time for other threads
        pros::delay(10);
    }
}


const GamePieceArray GetFrame() {
    frame_mutex.take();
	GamePieceArray cur_frame = g_frame;
    frame_ready = false;
    frame_mutex.give();
    return cur_frame;
}

std::optional<GamePieceData> GetObject(GamePiece piece) {
    frame_mutex.take();
    int idx = toIndex(piece);
    std::optional<GamePieceData> object = std::nullopt;
    if (idx < g_frame.size())
        object = g_frame[idx];

    frame_ready = false;
    frame_mutex.give();
    return object;
}

bool IsConnected() {
    frame_mutex.take();
    uint32_t last = last_update_time;
    frame_mutex.give();
    
    return pros::millis() - last < DISCONNECT_TIMEOUT_MS;
}



//0,150,200,0.95|1,320,100,0.88|2,480,350,0.79|3,50,450,0.92|3,600,50,0.85|3,400,250,0.72|3,250,50,0.98\n
//3,98,412,0.81|2,550,15,0.73|1,121,580,0.99|0,390,260,0.65|3,50,50,0.91|3,233,300,0.84|3,610,5,0.76\n
//3,500,105,0.77|3,145,390,0.82|1,288,250,0.91|0,55,15,0.96|3,405,450,0.68|2,200,88,0.73|3,330,330,0.89\n