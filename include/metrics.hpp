#pragma once
#include "arclengthSplining.hpp"
#include "odometry.hpp"



class PathFollowingTracking {
    public: 
        void update(const Pose& pose, const Sample& nearest);
        void reset();

        double getMaxCTE() const { return maxCTE; }
        double getAvgCTE() const { return avgCTE; }
        double getRMSCTE() const;
        double getFinalCTE() const { return finalCTE; }
        int    getSignChanges() const { return signChanges; }
        double getPosErrorAtPoint() const { return posErrorAtPoint; }

    private:    
        int n = 0;
        /*--------------Distance----------------*/

        //Maximum distance from path (cross track error)
        double maxCTE = 0.0;

        //Avg distance from path (cross track error)
        double avgCTE = 0.0;

        //Root Mean Square (RMS) distance from path (Cross Track Error)
        double rmsCTE = 0.0;


        /*--------------Position----------------*/

        //Final position error
        double finalCTE = 0.0;

        //Position error at a specific point
        double posErrorAtPoint = 0.0;

        /*--------------Misc--------------------*/

        //Error sign change count (when the sign of the position error changes, i.e. it crosses the path)
        int signChanges = 0;
        int lastSign = 0;
    
};

class HeadingAccuracyTracking {
    public: 
        void update(const Pose& pose, const Sample& nearest, double timestamp_s);
        void reset();

        double getMaxError() const { return maxError; }
        double getFinalError() const { return lastError; }
        double getRMSError() const;
        int    getOscillations() const { return oscillations; }

    private:
        
        double avgError = 0.0;
        int n = 0;
 
    /*--------------Error-------------------*/

    //Maximum heading error (degrees, be careful about backwards)
        double maxError = 0.0;

    //Final heading error (degrees)
        double lastError = 0.0;

    //Root Mean Square heading error
        double rmsError = 0.0;

    /*--------------Smoothness--------------*/
    
    //Heading oscillation count (when the heading rapidly switches directions)
        int oscillations = 0;
        int lastRateSign = 0;

    //Heading settling time (time to settle oscillation)
        double settlingTime     = -1.0;
        double settleStartTime  = -1.0;
        bool   inSettleWindow   = false;
        int    settleTickCount  = 0;
        static constexpr double SETTLE_RATE_THRESHOLD = 2.0;
        static constexpr int    SETTLE_TICKS_REQUIRED = 10;

};

class VelocityTracking {
    public: 
        void update(const Pose& pose, const Sample& nearest, double parallelVel);
        void reset();

        double getAvgVelocity() const { return avgVel; }
        double getMaxVelocity() const { return maxVel; }
    private:
        int n = 0;

    //AVG velocity
        double avgVel = 0.0;

    //Maximum Velocity
        double maxVel = 0.0;

};

class EndpointPerformanceTracking {
    public: 
        void record(const Pose& finalPose, const Sample& finalSample);
        void updateSettling(const Pose& pose, const Sample& goal, double timestamp_s);
        void reset();

        double getFinalPositionError() const { return finalPosError; }
        double getFinalHeadingError()  const { return finalHeadError; }
        double getTimeToSettle() const {return timeToSettle; }
        double getCorrectionCount() const { return correctionCount; }
        double getdistOvershoot() const {return distOvershoot; }
        double getHeadingOvershoot() const { return headingOvershoot; }
        
    private:

    /*--------------Position------------------*/

    //Final position error
        double finalPosError = 0.0;

    //Final heading error
        double finalHeadError = 0.0;

    /*--------------Stop Quality--------------*/

    //Time to settle
        double timeToSettle     = -1.0;

    //Number of corrections after reaching goal
        int    correctionCount  = 0;

        double settleStartTime  = -1.0;
        bool   settled          = false;
        static constexpr double POS_SETTLE_THRESH     = 0.5;
        static constexpr double HEADING_SETTLE_THRESH = 2.0;

    /*--------------Overshoot--------------*/

    //Distance overshoot
        double distOvershoot    = 0.0;

    //Heading overshoot
        double headingOvershoot = 0.0;

};

class TimingTracking {
    public: 
        void update(double cte, double headingErr, double timestamp_s, double cteThreshold = 2.0, double headingThreshold = 10.0);
        void start(double timestamp_s);
        void finish(double timestamp_s);
        void reset();

        double getTotalTime() const { return totalTime; }
        double getTimeAboveThreshold() const { return timeAboveThreshold; }
        double getStartTime() const { return startTime; }

    private:
    /*--------------Runtime--------------*/
        double startTime = 0.0;
        double lastTimestamp;
        bool started = false;

    //Total path completion time
        double totalTime = 0.0;

    //Time spent above threshold error
        double timeAboveThreshold = 0.0;

    /*--------------Settling--------------*/

    //Time to settle

};

class SmoothnessTracking {
    public: 
        void update(double parallelVel, double poseTheta_rad, double timestamp_s);
        void reset();

        double getAvgJerk()  const { return avgJerk; }
        double getPeakJerk() const { return peakJerk; }
        double getPeakSteeringJerk() const { return peakSteeringJerk; }
        double getAvgSteeringJerk() const { return avgSteeringJerk; }

    private:
        int n = 0;
        double lastVel      = 0.0;
        double lastAccel    = 0.0;
        double lastTime     = -1.0;
        bool initialized    = false;
        bool hasAccel       = false;



    //Average jerk
        double avgJerk = 0.0;

    //Peak jerk
        double peakJerk = 0.0;

    //Steering jerk
        double lastTheta        = 0.0;
        double lastOmega        = 0.0;
        double lastSteerAccel   = 0.0;
        double peakSteeringJerk = 0.0;
        double avgSteeringJerk  = 0.0;
        bool   hasOmega         = false;
        bool   hasSteerAccel    = false;

};

class MetricsSystem {
    public:
        void update(const Pose& pose, const Sample& nearestSample, double parallelVel, double timestamp_s);

        void startRun(double timestamp_s);

        void finishRun(const Pose& finalPose, const Sample& goalSample, double timestamp_s);

        PathFollowingTracking path;
        HeadingAccuracyTracking heading;
        VelocityTracking velocity;
        EndpointPerformanceTracking endpoint;
        TimingTracking timing;
        SmoothnessTracking smoothness;
};
