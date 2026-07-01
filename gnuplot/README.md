# Examples of brain side code:
```
//path logging
FieldLogger path_log(LoggerType::PATH, "test_path");
path_log.log(target_pose, current_pose, pros::millis()/1000);

//value logging
FieldLogger heading_log(LoggerType::VALUE, "test_path","velocity");
heading_log.log(heading, pros::millis()/1000);

//explicit flush
path_log.flush();

//early close
path_log.close();
```

# GNU Plot cpp code to run on computer

## Install GnuPlot locally

### Compile:
g++ -std=c++17 -o plotter plotter.cpp

### Create output directory
mkdir -p plots

### Run: (1st arg is SD card path, 2nd is output dir, 3rd and 4th are x and y offset)
./plotter /UNTITLED ./plots -48 -48

