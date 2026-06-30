# Examples of brain side code:
```
//path logging
FieldLogger pathLog(LoggerType::PATH, "test_path");
pathLog.log(targetPose, currentPose, 1.5);

//value logging
FieldLogger headingLog(LoggerType::VALUE, "test_path","velocity");
headingLog.log(heading, pros::millis()*1000);

//explicit flush
pathLog.flush();

//early close
pathLog.close();
```

# GNU Plot cpp code to run on computer

## Install GnuPlot locally

### Compile:
g++ -std=c++17 -o plotter plotter.cpp

### Create output directory
mkdir -p plots

### Run: (1st arg is SD card path, 2nd is output dir)
./plotter /UNTITLED ./plots

