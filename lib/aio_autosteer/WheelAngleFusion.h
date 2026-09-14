// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

// WheelAngleFusion.h - Virtual WAS (VWAS) using sensor fusion
#ifndef WHEEL_ANGLE_FUSION_H
#define WHEEL_ANGLE_FUSION_H

#include <Arduino.h>
#include "EventLogger.h"

// Math constants are already defined in Arduino.h
// DEG_TO_RAD and RAD_TO_DEG

// Forward declarations
class MotorDriverInterface;
class GNSSProcessor;
class IMUProcessor;

/**
 * WheelAngleFusion - Virtual Wheel Angle Sensor (VWAS)
 * 
 * Creates a virtual WAS by combining multiple sensor inputs to estimate 
 * steering angle. Uses an adaptive Kalman filter to fuse motor encoder 
 * data with GPS/INS heading rate.
 * 
 * This provides a software-based wheel angle sensor using existing hardware.
 * Based on proven algorithm from AOG_Teensy_UM98X project.
 */
class WheelAngleFusion {
public:
    // Configuration structure
    struct Config {
        // Vehicle parameters
        float wheelbase = 2.14f;          // Vehicle wheelbase in meters (was 2.5 - wrong;
                                           // confirmed 2.14 from real vehicle.xml export)
        float trackWidth = 1.25f;         // Vehicle track width in meters

        // Vehicle kinematic type - matches AgOpenGPS's own setVehicle_vehicleType
        // values (confirmed from real exported vehicle.xml files):
        //   0 = Standard (Ackermann front-wheel steering)
        //   2 = Articulated (center-pivot / "Knicklenker")
        // Other AgOpenGPS vehicleType values (e.g. Harvester, Trike) are not
        // handled here yet and fall back to the Standard (Ackermann) model.
        uint8_t vehicleType = 0;
        
        // Motor calibration
        // NOTE: countsPerDegree is a rough placeholder and is almost
        // certainly wrong for your specific steering linkage/gear ratio.
        // A wrong value here (wrong magnitude OR wrong sign) will make the
        // steering loop over- or under-react and can cause exactly the
        // "turns one way, overcorrects, slams into the stop" behaviour.
        // Verify/tune experimentally: turn the wheel a known number of
        // degrees by hand (motor disabled) and check the logged
        // "Encoder angle" value (LOG_DEBUG in updateEncoderAngle()).
        float countsPerDegree = 100.0f;   // Encoder counts per steering degree
        bool invertEncoder = false;       // Flip sign if angle moves the wrong way
        int32_t centerPosition = 32768;   // Encoder position when wheels straight
        float maxSteeringAngle = 40.0f;   // Maximum steering angle (degrees)
        
        // Kalman filter parameters
        float processNoise = 0.1f;        // Q - process noise covariance
        float measurementNoise = 1.0f;    // R - measurement noise covariance
        float initialUncertainty = 10.0f; // P - initial error covariance
        
        // Fusion parameters
        float minSpeedForGPS = 0.5f;      // Minimum speed for GPS fusion (m/s) - kept for now per user request
        float maxHeadingRate = 50.0f;     // Maximum valid heading rate (deg/s)
        uint16_t varianceBufferSize = 50; // Size of variance calculation buffer

        // --- Encoder-dominant drift correction (replaces per-cycle Kalman blend) ---
        // Design follows the proven community approach (Cerea, AOG_Teensy_UM98X,
        // 87yj/AgOpen_Keya_VirtualWAS): the encoder is ALWAYS the primary,
        // instantaneous angle source. GPS/dual-heading is only ever used to
        // slowly trim a long-term offset (encoderOffset) - never to directly
        // replace or blend into the instantaneous angle. This avoids injecting
        // noisy single-sample heading-rate measurements straight into the
        // steering command.
        bool preferDualHeading = true;    // Use UM982 dual-antenna heading (gpsData.dualHeading)
                                           // instead of single-antenna course-over-ground
                                           // (gpsData.headingTrue) whenever available. Dual
                                           // heading is a true compass reading and does not
                                           // require vehicle motion to be accurate.
        uint8_t minHeadingQuality = 2;    // Minimum dualHeading quality to trust (HPR: 0=no fix,
                                           // 1=single, 2=float RTK, 4=RTK fixed)
        float minDistanceForCurvature = 0.10f; // Minimum distance (m) travelled between samples
                                           // before trusting the heading-derived curvature.
                                           // Curvature is computed as headingDelta/distance
                                           // (geometric, speed-independent) rather than
                                           // headingRate/speed - this removes a second,
                                           // independently-noisy division term.
        float gpsAngleSmoothingTau = 1.0f;  // EMA time constant (s) for smoothing the raw
                                           // heading-derived angle before it's used to trim
                                           // the encoder offset.
        float maxDriftCorrectionRate = 0.5f; // Maximum encoderOffset change (deg/s). Bounds
                                           // how fast GPS can ever pull the reported angle,
                                           // regardless of how large the disagreement is.
        float driftInnovationRejectThreshold = 15.0f; // If the smoothed GPS-derived angle
                                           // disagrees with the encoder by more than this many
                                           // degrees, treat it as an outlier/real manoeuvre and
                                           // skip the drift correction entirely this cycle.
        float maxPlausibleSpeed = 5.0f;   // m/s. If distance/time between the current and a
                                           // looked-up history sample implies a speed above this,
                                           // treat it as a GPS jump/multipath glitch and reject
                                           // the sample (guards against noisy low-cost GPS
                                           // occasionally reporting a spurious position jump).

        // Sensor selection
        bool useIMUHeadingRate = false;   // Use IMU instead of GPS for heading rate
        bool enableDriftCompensation = true; // Enable encoder drift compensation
    };
    
    // Constructor
    WheelAngleFusion();
    ~WheelAngleFusion() = default;
    
    // Initialization
    // motor: any MotorDriverInterface with hasPositionFeedback()==true
    //        (TractorCANDriver with Keya, or KeyaSerialDriver)
    bool init(MotorDriverInterface* motor, GNSSProcessor* gnss, IMUProcessor* imu);
    
    // Configuration
    void setConfig(const Config& cfg) { config = cfg; }
    Config& getConfig() { return config; }
    const Config& getConfig() const { return config; }
    
    // Main update function - call at 100Hz
    void update(float dt);
    
    // Get fusion results
    float getFusedAngle() const { return fusedAngle; }
    float getEncoderOffset() const { return encoderOffset; }
    float getPredictedAngle() const { return predictedAngle; }
    float getGPSAngle() const { return gpsAngle; }
    float getEncoderAngle() const { return encoderAngle; }
    
    // Get quality metrics
    float getUncertainty() const { return uncertainty; }
    float getMeasurementVariance() const { return measurementVariance; }
    float getKalmanGain() const { return kalmanGain; }
    
    // Health and status
    bool isHealthy() const;
    bool hasValidGPSAngle() const { return gpsAngleValid; }
    uint32_t getLastUpdateTime() const { return lastUpdateTime; }
    
    // Calibration
    void startCalibration();
    void stopCalibration();
    bool isCalibrating() const { return calibrationMode; }
    void setEncoderCenter(); // Set current position as center (0 degrees)

    // Set encoderOffset directly to a specific value (degrees, in raw
    // encoderAngle scale) rather than "current position". Used by the
    // lock-to-lock endstop calibration sweep, which computes the midpoint
    // between two recorded extremes without physically returning the
    // motor to that midpoint first.
    void setEncoderOffsetDirect(float offsetDegrees) { encoderOffset = offsetDegrees; }

    // Recalibrate countsPerDegree from a measured lock-to-lock sweep.
    // measuredSwingDegrees: |rightAngle - leftAngle| as read via
    // getEncoderAngle() at each end-stop, using the CURRENT (possibly
    // wrong) countsPerDegree.
    // trueSwingDegrees: the actual mechanical full lock-to-lock angle
    // (e.g. 2x the vehicle's maxSteerAngle from AgOpenGPS), which the user
    // must have entered correctly beforehand.
    void recalibrateCountsPerDegree(float measuredSwingDegrees, float trueSwingDegrees) {
        if (trueSwingDegrees > 0.01f && measuredSwingDegrees > 0.01f) {
            config.countsPerDegree = config.countsPerDegree *
                                      (measuredSwingDegrees / trueSwingDegrees);
        }
    }
    
    // Reset and recovery
    void reset();
    void resetDriftCompensation();
    
private:
    // Sensor interfaces
    MotorDriverInterface* keyaDriver;  // name kept for minimal diff; any driver with position feedback
    GNSSProcessor* gnssProcessor;
    IMUProcessor* imuProcessor;
    
    // Configuration
    Config config;
    
    // Kalman filter state
    float fusedAngle;        // X - Current angle estimate
    float predictedAngle;    // Xp - Predicted angle
    float uncertainty;       // P - Estimation error covariance
    float kalmanGain;        // K - Kalman gain
    
    // Sensor angles
    float encoderAngle;      // Angle from motor encoder
    float gpsAngle;          // Angle from GPS heading rate
    bool gpsAngleValid;      // Is GPS angle valid this update
    
    // Motor position tracking
    int32_t lastEncoderPosition;
    float encoderOffset;      // Offset for centering (degrees). Was int32_t - a bug, since
                               // it silently truncated fractional-degree offsets.
    
    // GPS angle calculation
    float vehicleSpeed;      // Current speed (m/s), smoothed 80/20 EMA (matches AgOpenGPS)
    bool vehicleSpeedInit;
    float headingRate;       // Rate of heading change (deg/s) - diagnostic only now
    float lastHeading;       // Previous heading for rate calculation
    // Ring buffer of recent GPS samples (position + heading + time), used to
    // look BACK for a sample far enough away (>= minDistanceForCurvature)
    // instead of waiting forward for enough new distance to accumulate.
    // This gives a fresh candidate on every GPS update instead of only
    // once every ~minDistanceForCurvature/speed seconds - important at
    // very low speed (~0.04 m/s => ~2.5s between updates otherwise).
    static const uint16_t GPS_HISTORY_SIZE = 300; // ~15-30s of history at 10-20Hz
    struct GPSHistorySample {
        double lat;
        double lon;
        float heading;
        uint32_t timeMs;
    };
    GPSHistorySample gpsHistory[GPS_HISTORY_SIZE];
    uint16_t gpsHistoryHead;   // next write index
    uint16_t gpsHistoryCount;  // number of valid entries (caps at GPS_HISTORY_SIZE)
    float lastDistanceTraveled;  // Distance (m) since last curvature sample - diagnostic
    bool usingDualHeadingNow;    // Diagnostic: which heading source was used last cycle
    uint8_t lastHeadingQualityUsed; // Diagnostic
    float gpsAngleSmoothed;      // EMA-smoothed heading-derived angle
    bool gpsAngleSmoothedInit;
    float gpsIntegratedAngle;    // Articulated mode only: currentHeading minus
                                  // referenceHeadingArticulated (wrapped), NOT an
                                  // incremental sum - see referenceHeadingArticulated.
    bool gpsIntegratedAngleInit;
    float referenceHeadingArticulated; // Front-unit heading captured once at
                                  // calibration/first-fix, used as the articulated
                                  // "straight ahead" reference. Computing
                                  // (currentHeading - this) directly, rather than
                                  // summing many small deltas, avoids double-
                                  // counting when history lookback windows overlap.
    bool haveReferenceHeadingArticulated;
    uint32_t lastDriftCorrectionTime; // For rate-limiting using actual elapsed
                                  // time between GPS-valid samples, not the
                                  // fixed per-cycle dt (GPS updates can be much
                                  // rarer than the 100Hz fusion loop, especially
                                  // at very low speed / large minDistanceForCurvature)
    
    // Adaptive variance calculation
    float measurementVariance;
    float* varianceBuffer;   // Circular buffer for variance calculation
    uint16_t varianceIndex;
    uint16_t varianceCount;
    
    // Drift compensation
    float encoderDrift;      // Accumulated drift (degrees)
    float driftRate;         // Drift rate (degrees/second)
    uint32_t driftStartTime;
    
    // Timing
    uint32_t lastUpdateTime;
    uint32_t lastGPSTime;
    
    // Calibration
    bool calibrationMode;
    float calibrationMinAngle;
    float calibrationMaxAngle;
    int32_t calibrationMinPosition;
    int32_t calibrationMaxPosition;
    
    // Private methods
    void updateEncoderAngle();
    void updateGPSAngle();
    void updateKalmanFilter(float dt);
    void updateVariance();
    void updateDriftCompensation(float dt);
    float calculateGPSAngleFromHeadingRate(float headingRate, float speed);
    bool isValidGPSConditions() const;
};

// Global instance pointer for external access
extern WheelAngleFusion* wheelAngleFusionPtr;

#endif // WHEEL_ANGLE_FUSION_H