#include <chrono>
#include <thread>

void main (void) {


    while (true) {



        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}