// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

#ifndef AUTOSTEER_PROCESSOR_H
#define AUTOSTEER_PROCESSOR_H

#include <Arduino.h>
// External pointers
class ADProcessor;
extern ADProcessor adProcessor;

class KickoutMonitor;

// PGN data is parsed directly to ConfigManager
// No intermediate structs needed

class AutosteerProcessor {
private:
    static AutosteerProcessor* instance;

    // Private constructor for singleton
    AutosteerProcessor();
    
    // Configuration storage
    
    // State tracking
    bool autosteerEnabled = false;
    float targetAngle = 0.0f;
    uint32_t lastPGN254Time = 0;
    
    // PGN 254 data
    float vehicleSpeed = 0.0f;      // km/h
    bool guidanceActive = false;     // Guidance line active
    int8_t crossTrackError = 0;      // cm off line
    uint16_t machineSections = 0;    // 16 section states
    
    // Button state tracking
    bool physicalButtonState = false;     // Current physical button state
    bool lastPhysicalButtonState = false; // Previous physical button state
    uint32_t lastButtonDebounceTime = 0;  // Debounce timer
    static constexpr uint32_t DEBOUNCE_DELAY = 50; // 50ms debounce
    
    // Autosteer loop timing - Now handled by SimpleScheduler at 100Hz
    
    // Steer state management
    uint8_t steerState = 1;              // 0 = steering active, 1 = steering inactive
    bool prevGuidanceStatus = false;     // Previous guidance status from AgOpenGPS
    bool guidanceStatusChanged = false;  // Flag for guidance status change
    
    // Motor control
    float currentAngle = 0.0f;           // Current WAS angle
    float actualAngle = 0.0f;            // Ackerman-corrected angle
    int16_t motorPWM = 0;                // Current motor PWM command (-255 to +255)
    
    // Watchdog
    uint32_t lastCommandTime = 0;        // Last time we received PGN 254
    static constexpr uint32_t WATCHDOG_TIMEOUT = 2000; // 2 seconds
    
    // Kickout
    uint32_t kickoutTime = 0;            // Time of last kickout
    static constexpr uint32_t KICKOUT_COOLDOWN_MS = 2000; // 2 second cooldown
    KickoutMonitor* kickoutMonitor = nullptr;
    bool lastKickoutStateForTelemetry = false;
    uint32_t pwmFreezeStartTime = 0;
    uint8_t frozenPwmDisplay = 0;
    static constexpr uint32_t POST_KICKOUT_TELEMETRY_MS = 3000;
    
    // Soft-start motor control
    enum class MotorState {
        DISABLED,
        SOFT_START,
        SOFT_ACCEL,
        NORMAL_CONTROL
    };

    MotorState motorState = MotorState::DISABLED;
    uint32_t softStartBeginTime = 0;
    float softStartRampValue = 0.0f;

    // Soft-start configuration constants
    static constexpr uint8_t DIRECTION_CHANGE_THRESHOLD = 20;  // PWM threshold for direction change detection
    static constexpr float ACCEL_THRESHOLD_RATIO = 0.33f;      // 1/3 of PWM range for acceleration detection

    // Soft-start parameters (configurable via Web UI)
    uint16_t softStartDurationMs = 500;     // Duration of soft-start ramp (400-1000ms range)
    uint16_t softAccelDurationMs = 250;     // Duration of soft-acceleration ramp (150-500ms range)
    bool useSineRamp = false;                // Use sine curve (true) or linear ramp (false)
    int16_t lastPwmDrive = 0;              // Last raw pwmDrive before ramp/inversion, for direction tracking
    float filteredCommandPwm = 0.0f;       // Global filtered PWM command applied to all motor drivers
    
    // Deferred disarm for AOG OSB handshake
    // When valve/motor not ready, we briefly arm so AOG sees the state change,
    // then disarm after 200ms so AOG can properly toggle the OSB off.
    bool pendingDisarm = false;
    uint32_t pendingDisarmTime = 0;
    static constexpr uint32_t DISARM_HANDSHAKE_MS = 200;

    // Link state tracking
    bool linkWasDown = false;               // Track if link was down
    
    // Motor config change tracking
    uint8_t previousMotorConfig = 0xFF;     // Previous motor config byte
    int8_t previousCytronDriver = -1;       // Previous Cytron bit state
    bool motorConfigInitialized = false;    // Track if we've initialized from EEPROM

    // --- VWAS endstop calibration sweep ---
    // Drives the motor slowly to each mechanical steering stop to find the
    // true centre position (and optionally recalibrate countsPerDegree),
    // reusing the existing KickoutMonitor slip/current detection to know
    // when a stop has been reached - no new thresholds needed.
    enum class VWASCalibState {
        IDLE,
        SWEEP_LEFT,
        PAUSE_AT_LEFT,
        SWEEP_RIGHT,
        PAUSE_AT_RIGHT,
        DONE,
        ABORTED
    };
    VWASCalibState vwasCalibState = VWASCalibState::IDLE;
    uint32_t vwasCalibStateStartTime = 0;
    float vwasCalibLeftAngle = 0.0f;
    float vwasCalibRightAngle = 0.0f;
    static constexpr int16_t VWAS_CALIB_SWEEP_PWM = 60;   // Slow, gentle - well below normal highPWM
    static constexpr uint32_t VWAS_CALIB_TIMEOUT_MS = 15000; // Abort if no stop found in time
    static constexpr uint32_t VWAS_CALIB_PAUSE_MS = 800;  // Brief pause/reversal settle time
    void processVWASCalibration();
    
    
public:
    // Singleton access
    static AutosteerProcessor* getInstance();
    
    // Initialization
    bool init();
    void process();
    void initializeFusion();  // Initialize sensor fusion separately
    
    // PGN handlers
    void handleBroadcastPGN(uint8_t pgn, const uint8_t* data, size_t len);
    void handleSteerConfig(uint8_t pgn, const uint8_t* data, size_t len);
    void handleSteerSettings(uint8_t pgn, const uint8_t* data, size_t len);
    void handleSteerData(uint8_t pgn, const uint8_t* data, size_t len);
    
    // Send replies
    void sendHelloReply();
    void sendScanReply(bool extended);
    
    // Send PGN 253 status to AgOpenGPS
    void sendPGN253();
    
    // Motor control
    void updateMotorControl();
    void emergencyStop();
    bool shouldSteerBeActive() const;

    // --- VWAS calibration (web UI buttons) ---
    // "Geradeausstellung setzen": call with the wheels physically straight.
    // Safe, instant, does not move the motor.
    bool vwasSetCenterNow();

    // Lock-to-lock endstop sweep: physically drives the motor to both
    // mechanical stops to find true centre (and recalibrate
    // countsPerDegree, if maxSteeringAngle is set correctly). MOVES THE
    // STEERING. Refuses to start while AgOpenGPS guidance is active.
    bool vwasStartEndstopCalibration();
    void vwasAbortCalibration();
    const char* vwasCalibrationStateString() const;
    bool vwasCalibrationInProgress() const {
        return vwasCalibState != VWASCalibState::IDLE &&
               vwasCalibState != VWASCalibState::DONE &&
               vwasCalibState != VWASCalibState::ABORTED;
    }
    float vwasCalibLeftAngleResult() const { return vwasCalibLeftAngle; }
    float vwasCalibRightAngleResult() const { return vwasCalibRightAngle; }

    // VWAS config persistence (web UI save / boot load)
    void saveVWASConfig();
    void loadVWASConfig();
    
    // Static callback wrapper for PGN registration
    static void handlePGNStatic(uint8_t pgn, const uint8_t* data, size_t len);
    
    
    // Public getters for state
    bool isEnabled() const { return autosteerEnabled; }
    float getTargetAngle() const { return targetAngle; }

    // Public getter for PGN254 vehicle speed 
    float getVehicleSpeed() const { return vehicleSpeed; }

    // Soft-start configuration
    uint16_t getSoftStartDuration() const { return softStartDurationMs; }
    void setSoftStartDuration(uint16_t durationMs) {
        softStartDurationMs = constrain(durationMs, 0, 1000); // Max 1000ms
    }

    uint16_t getSoftAccelDuration() const { return softAccelDurationMs; }
    void setSoftAccelDuration(uint16_t durationMs) {
        softAccelDurationMs = constrain(durationMs, 0, 500); // Max 500ms
    }

    bool getUseSineRamp() const { return useSineRamp; }
    void setUseSineRamp(bool useSine) { useSineRamp = useSine; }
};

// Global instance
extern AutosteerProcessor autosteerProcessor;

#endif // AUTOSTEER_PROCESSOR_H