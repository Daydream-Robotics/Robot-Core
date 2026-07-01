#pragma once
#include "sd_card_logging.hpp"
#include "arclengthSplining.hpp"
#include <vector>
#include <cmath>

//logger type enum
enum class LoggerType {
    PATH, //logs (target trajectory, actual trajectory, and error over time)
    VALUE //logs a value over time
};

class FieldLogger: public Logger {
    public:
        FieldLogger(LoggerType type, const char* baseName, const char* label = nullptr, bool overwrite = true) {
            //buffer for file paths
            char filename[64];
            //store logging mode
            log_type = type;
            //value used to reduce write overhead
            flush_counter = 0;


            switch(log_type){
                //path mode
                case LoggerType::PATH:
                //if overwrite is enabled delete old target, actual, and error .dat files with the same basenam
                    if (overwrite) {
                        snprintf(filename, sizeof(filename), "/usd/%s_target.dat", baseName);
                        remove(filename);
                        snprintf(filename, sizeof(filename), "/usd/%s_actual.dat", baseName);
                        remove(filename);
                        snprintf(filename, sizeof(filename), "/usd/%s_error.dat", baseName);
                        remove(filename);
                    }

                    //open target path log
                    snprintf(filename, sizeof(filename), "/usd/%s_target.dat", baseName);
                    target_pos_file = fopen(filename, "w");

                    //open actual path log
                    snprintf(filename, sizeof(filename), "/usd/%s_actual.dat", baseName);
                    actual_pos_file = fopen(filename, "w");

                    //open tracking error log
                    snprintf(filename, sizeof(filename), "/usd/%s_error.dat", baseName);
                    error_pos_file = fopen(filename, "w");
                    break;
                
                //value mode
                case LoggerType::VALUE:
                    //makes a fallback label
                    const char* safe_label = label ? label : "value";
                    //if overwrite is enabled delete old value .dat file of same name
                    if (overwrite) {
                        snprintf(filename, sizeof(filename), "/usd/%s_%s.dat", baseName, safe_label);
                        remove(filename);
                    }

                    //open value log file
                    snprintf(filename, sizeof(filename), "/usd/%s_%s.dat", baseName, safe_label);
                    value_file = fopen(filename, "w");

                    //header for plotting
                    fprintf(value_file, "# t value\n");

                    //ensure header is written immediatly
                    flush();

                    break;
            }
        }

        //destructor
        ~FieldLogger() {
            close();
        }

        //unified log function for LoggerType::PATH
        void log(const Waypoint& target, const Waypoint& current, double t) {
            if (log_type == LoggerType::PATH) {
                logPath(target, current, t);
            }
        }

        //unified log function for LoggerType::VALUE
        void log(double value, double t) {
            if (log_type == LoggerType::VALUE) {
                logValue(value, t);
            }
        }

        //flush all open files
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

        //close all files safely
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

        //prevent copying because file handles can't be duplicated
        FieldLogger(const FieldLogger&) = delete;
        FieldLogger& operator=(const FieldLogger&) = delete;

    private:
        //file handles for PATH mode
        FILE* target_pos_file = nullptr;
        FILE* actual_pos_file = nullptr;
        FILE* error_pos_file = nullptr;

        //file handle for VALUE mode
        FILE* value_file = nullptr;

        //current logging mode
        LoggerType log_type;
        //value used to reduce write overhead
        int flush_counter;

        //function to log target trajectory, actual trajectory, and error over time in .dat files
        void logPath(const Waypoint& target, const Waypoint& current, double time) {
            //log type check
            if(log_type != LoggerType::PATH ) {
                return;
            }

            if (target_pos_file) {
                //log target position to sd card (time, x, y)
                fprintf(target_pos_file, "%.4f %.4f %.4f\n", time, target.x, target.y);
            }
            if (actual_pos_file) {
                //log actual robot position to sd card (time, x, y)
                fprintf(actual_pos_file, "%.4f %.4f %.4f\n", time, current.x, current.y);
            }
            
            //get tracking error
            double dx = target.x - current.x;
            double dy = target.y - current.y;
            double error = std::hypot(dx, dy);
            if (error_pos_file) {
                //log scalar error magnitude to sd card(time, error)
                fprintf(error_pos_file, "%.4f %.4f\n", time, error);
            }
            
            //batch the flushes to reduce overhead
            if (++flush_counter >= 10) {
                flush();
                flush_counter = 0;
            }
        }

        //valeu logging implementation
        void logValue(double value, double time) {
            //log type check, and check for valid file
            if((log_type != LoggerType::VALUE) || !value_file) {
                return;
            }

            //log value to sd card(time, value)
            fprintf(value_file, "%.4f %.4f\n", time, value);
            
            //batch the flushes to reduce overhead
            if (++flush_counter >= 10) {
                flush();
                flush_counter = 0;
            }
        }
};

