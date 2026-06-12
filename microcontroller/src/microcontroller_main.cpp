#include <chrono>
#include <thread>

int main (void) {


    while (true) {



        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}