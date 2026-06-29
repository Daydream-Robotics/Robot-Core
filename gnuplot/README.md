# Examples pf brain side code:
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


