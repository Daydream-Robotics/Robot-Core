#pragma once
#include "sd_card_logging.hpp"
#include "arclengthSplining.hpp"
#include <vector>
#include <cmath>

enum class LoggerType {
    PATH, 
    VALUE
};

class FieldLogger: public Logger {
    public:
        FieldLogger(LoggerType type, const char* baseName, const char* label = nullptr, bool overwrite = true) {
            char filename[64];
            log_type = type;
            flush_counter = 0;
            switch(log_type){
                case LoggerType::PATH:
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
                    break;
                
                case LoggerType::VALUE:
                    const char* safe_label = label ? label : "value";
                    if (overwrite) {
                        snprintf(filename, sizeof(filename), "/usd/%s_%s.dat", baseName, safe_label);
                        remove(filename);
                    }
                    snprintf(filename, sizeof(filename), "/usd/%s_%s.dat", baseName, safe_label);
                    value_file = fopen(filename, "w");
                    fprintf(value_file, "# t value\n");
                    flush();
                    break;
            }
        }

        ~FieldLogger() {
            close();
        }

        void log(const Waypoint& target, const Waypoint& current, double t) {
            if (log_type == LoggerType::PATH) {
                logPath(target, current, t);
            }
        }

        void log(double value, double t) {
            if (log_type == LoggerType::VALUE) {
                logValue(value, t);
            }
        }

        void flush() {
            switch(log_type){
                case LoggerType::PATH:
                    if (target_pos_file) {
                        fflush(target_pos_file);
                    }
                    if (actual_pos_file) {
                        fflush(actual_pos_file);
                    }
                    if (error_pos_file) {
                        fflush(error_pos_file);
                    }
                    break;
                case LoggerType::VALUE:
                    if (value_file) {
                        fflush(value_file);
                    }
                    break;
            }
        }

        void close() {
            switch(log_type){
                case LoggerType::PATH:
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
                    break;
                case LoggerType::VALUE:
                    if (value_file) { 
                        fclose(value_file); 
                        value_file = nullptr; 
                    }
                    break;
            }
        }

        FieldLogger(const FieldLogger&) = delete;
        FieldLogger& operator=(const FieldLogger&) = delete;

    private:
        FILE* target_pos_file = nullptr;
        FILE* actual_pos_file = nullptr;
        FILE* error_pos_file = nullptr;
        FILE* value_file = nullptr;
        LoggerType log_type;
        int flush_counter;


        void logPath(const Waypoint& target, const Waypoint& current, double t) {
            if(log_type != LoggerType::PATH ) {
                return;
            }

            if (target_pos_file) {
                fprintf(target_pos_file, "%.4f %.4f %.4f\n", t, target.x, target.y);
            }
            if (actual_pos_file) {
                fprintf(actual_pos_file, "%.4f %.4f %.4f\n", t, current.x, current.y);
            }
            
            double dx = target.x - current.x;
            double dy = target.y - current.y;
            double error = std::hypot(dx, dy);
            if (error_pos_file) {
                fprintf(error_pos_file, "%.4f %.4f\n", t, error);
            }
            
            if (++flush_counter >= 10) {
                flush();
                flush_counter = 0;
            }
        }

        void logValue(double value, double t) {
            if((log_type != LoggerType::VALUE) || !value_file) {
                return;
            }
            fprintf(value_file, "%.4f %.4f\n", t, value);
            
            if (++flush_counter >= 10) {
                flush();
                flush_counter = 0;
            }
        }
};

