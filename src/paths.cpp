#include "paths.hpp"
#include "main.h"

std::vector<ALS_Path> buildAllPaths(double sampleSpacing) {
   
    std::vector<ALS_Path> als_paths;

    for (const auto& raw_path : raw_paths) {
        ALS_Path temp_als_path;
        temp_als_path.buildFromPoints(raw_path, sampleSpacing); //(path id, sample spacing)
	    temp_als_path.isValid() ? pros::lcd::print(2, "Path build success") : pros::lcd::print(2, "Path build failure");
        
        als_paths.push_back(temp_als_path);
    }

    return als_paths;
}
