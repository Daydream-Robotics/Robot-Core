#pragma once
#include "sd_card_logging.hpp"
#include <vector>

struct FieldPathPoint {
    double x; //[in]
    double y; //[in]
    double t; //[s]
};

class FieldLogger: public Logger {
    public:
        static FieldLogger& getInstance() {
            static FieldLogger instance;
            return instance;
        }

        bool startSession (const char* baseName, bool overwrite = true) {
            if (!sdAvailable) {
                return false;
            }
            closeAll();

            char filename[64];

            if (overwrite) {
                snprintf(filename, sizeof(filename), "/usd/%s_target.dat", baseName);
                remove(filename);
                snprintf(filename, sizeof(filename), "/usd/%s_actual.dat", baseName);
                remove(filename);
                snprintf(filename, sizeof(filename), "/usd/%s_error.dat", baseName);
                remove(filename);
            }

            snprintf(filename, sizeof(filename), "/usd/%s_target.dat", baseName);
            target_pos_file = fopen(filename, "w");

            snprintf(filename, sizeof(filename), "/usd/%s_actual.dat", baseName);
            actual_pos_file = fopen(filename, "w");

            snprintf(filename, sizeof(filename), "/usd/%s_error.dat", baseName);
            error_pos_file = fopen(filename, "w");
        }

        void closeAll() {
            if (target_pos_file) { 
                fclose(target_pos_file); 
                target_pos_file = nullptr; 
            }
            if (actual_pos_file) { 
                fclose(actual_pos_file); 
                actual_pos_file = nullptr; 
            }
            if (error_pos_file) { 
                fclose(error_pos_file); 
                error_pos_file = nullptr; 
            }
        }

    private:
        FILE* target_pos_file;
        FILE* actual_pos_file;
        FILE* error_pos_file;
};

