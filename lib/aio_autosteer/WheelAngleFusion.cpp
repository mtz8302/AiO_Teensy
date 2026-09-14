// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

// WheelAngleFusion.cpp - Implementation of sensor fusion for wheel angle
#include "WheelAngleFusion.h"
#include "MotorDriverInterface.h"
#include "GNSSProcessor.h"
#include "IMUProcessor.h"
#include <cmath>
#include <algorithm> // For constrain, fmax, fmin

// Global instance pointer
WheelAngleFusion* wheelAngleFusionPtr = nullptr;

// External sensor instances
extern GNSSProcessor* gnssProcessorPtr;
extern IMUProcessor imuProcessor;

static float normalizeAngle(float a) { // Helper function for angle normalization
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

WheelAngleFusion::WheelAngleFusion() :
    keyaDriver(nullptr),
    gnssProcessor(nullptr),
    imuProcessor(nullptr),
    fusedAngle(0.0f),
    predictedAngle(0.0f),
    uncertainty(10.0f),
    kalmanGain(0.0f),
    encoderAngle(0.0f),
    gpsAngle(0.0f),
    gpsAngleValid(false),
    lastEncoderPosition(0),
    encoderOffset(0.0f),
    vehicleSpeed(0.0f),
    vehicleSpeedInit(false),
    headingRate(0.0f),
    lastHeading(0.0f),
    gpsHistoryHead(0),
    gpsHistoryCount(0),
    lastDistanceTraveled(0.0f),
    usingDualHeadingNow(false),
    lastHeadingQualityUsed(0),
    gpsAngleSmoothed(0.0f),
    gpsAngleSmoothedInit(false),
    gpsIntegratedAngle(0.0f),
    gpsIntegratedAngleInit(false),
    referenceHeadingArticulated(0.0f),
    haveReferenceHeadingArticulated(false),
    lastDriftCorrectionTime(0),
    measurementVariance(1.0f),
    varianceBuffer(nullptr),
    varianceIndex(0),
    varianceCount(0),
    encoderDrift(0.0f),
    driftRate(0.0f),
    driftStartTime(0),
    lastUpdateTime(0),
    lastGPSTime(0),
    calibrationMode(false),
    calibrationMinAngle(-40.0f),
    calibrationMaxAngle(40.0f),
    calibrationMinPosition(0),
    calibrationMaxPosition(65535)
{
    // Set global pointer
    wheelAngleFusionPtr = this;
}

bool WheelAngleFusion::init(MotorDriverInterface* motor, GNSSProcessor* gnss, IMUProcessor* imu) {
    LOG_INFO(EventSource::AUTOSTEER, "Initializing Virtual WAS (VWAS)");
    
    // Store sensor interfaces
    keyaDriver = motor;
    gnssProcessor = gnss;
    imuProcessor = imu;
    
    // Validate interfaces
    if (!keyaDriver) {
        LOG_ERROR(EventSource::AUTOSTEER, "VWAS: No motor driver provided");
        return false;
    }

    if (!keyaDriver->hasPositionFeedback()) {
        LOG_ERROR(EventSource::AUTOSTEER, "VWAS: Motor driver (%s) has no position feedback",
                  keyaDriver->getTypeName());
        return false;
    }
    
    if (!gnssProcessor && !imuProcessor) {
        LOG_ERROR(EventSource::AUTOSTEER, "VWAS: No heading rate source (GPS or IMU)");
        return false;
    }
    
    // Allocate variance buffer
    if (varianceBuffer) {
        delete[] varianceBuffer;
    }
    varianceBuffer = new float[config.varianceBufferSize];
    if (!varianceBuffer) {
        LOG_ERROR(EventSource::AUTOSTEER, "VWAS: Failed to allocate variance buffer");
        return false;
    }
    
    // Initialize variance buffer
    for (uint16_t i = 0; i < config.varianceBufferSize; i++) {
        varianceBuffer[i] = 0.0f;
    }
    
    // Initialize Kalman filter state
    fusedAngle = 0.0f;
    predictedAngle = 0.0f;
    encoderAngle = 0.0f;
    gpsAngle = 0.0f;
    uncertainty = config.initialUncertainty;
    
    // Initialize timing
    lastUpdateTime = millis();
    lastGPSTime = millis();
    driftStartTime = millis();
    
    LOG_INFO(EventSource::AUTOSTEER, "Virtual WAS initialized successfully");
    LOG_INFO(EventSource::AUTOSTEER, "  Wheelbase: %.2f m", config.wheelbase);
    LOG_INFO(EventSource::AUTOSTEER, "  Counts/degree: %.1f", config.countsPerDegree);
    LOG_INFO(EventSource::AUTOSTEER, "  Min GPS speed: %.1f m/s", config.minSpeedForGPS);
    
    return true;
}

void WheelAngleFusion::update(float dt) {
    // Update timing
    uint32_t now = millis();
    lastUpdateTime = now;
    
    // Update sensor angles
    updateEncoderAngle();
    updateGPSAngle();
    
    // Run Kalman filter
    updateKalmanFilter(dt);
    
    // Update adaptive parameters
    if (isValidGPSConditions()) {
        updateVariance();
    }
    
    // Update drift compensation
    if (config.enableDriftCompensation) {
        updateDriftCompensation(dt);
    }

    // Comprehensive diagnostic log: every input/output of the GPS+encoder
    // fusion in one line, rate-limited so it doesn't flood the log.
    // Compare lastHeading (heading used in the previous cycle's delta) is
    // not stored separately from currentHeading, so we log currentHeading
    // (== gpsData.headingTrue at the time updateGPSAngle() ran this cycle).
    {
        static uint32_t lastDiagLogTime = 0;
        if (now - lastDiagLogTime > 200) {
            lastDiagLogTime = now;
            uint8_t fixQ = gnssProcessor ? gnssProcessor->getData().fixQuality : 0;
            LOG_INFO(EventSource::AUTOSTEER,
                "VWAS diag: type=%d heading=%.2f\u00B0(%s q=%d) headingRate=%.2f\u00B0/s "
                "speed=%.3fm/s fixQ=%d gpsHist=%d/%d dist=%.2fm "
                "gpsValid=%d gpsAngle=%.2f\u00B0 enc=%.2f\u00B0 off=%.2f\u00B0 pred=%.2f\u00B0 "
                "fused=%.2f\u00B0",
                config.vehicleType, lastHeading, usingDualHeadingNow ? "dual" : "COG",
                lastHeadingQualityUsed, headingRate, vehicleSpeed, fixQ,
                gpsHistoryCount, GPS_HISTORY_SIZE, lastDistanceTraveled,
                gpsAngleValid ? 1 : 0, gpsAngle, encoderAngle, encoderOffset, predictedAngle,
                fusedAngle);
        }
    }
}

void WheelAngleFusion::updateEncoderAngle() {
    if (!keyaDriver) {
        encoderAngle = 0.0f;
        return;
    }
    
    // Get position delta from motor
    int32_t deltaPosition = keyaDriver->getPositionDelta();
    
    // Convert delta to degrees
    float deltaAngle = (float)deltaPosition / config.countsPerDegree;
    if (config.invertEncoder) {
        deltaAngle = -deltaAngle;
    }
    
    // Update encoder angle (accumulate changes). This is the RAW
    // accumulator - it must NEVER have encoderOffset subtracted back into
    // it here. Offset subtraction happens only in updateKalmanFilter()'s
    // predictedAngle/fusedAngle calculation. Doing it here too caused a
    // severe bug: every cycle re-subtracted encoderOffset from
    // encoderAngle and wrote the result back, so a single "set centre"
    // action made encoderAngle spiral to the +/-maxSteeringAngle clamp
    // within a fraction of a second (100 cycles/sec x re-subtracting the
    // whole offset each time).
    encoderAngle += deltaAngle;
    
    // Log every raw movement (rate-limited) so direction/scale can be verified by hand-turning
    // the wheel with the motor disabled: rawDelta sign/magnitude should match the physical turn,
    // and deltaAngle should flip sign if invertEncoder is wrong.
    if (deltaPosition != 0) {
        static uint32_t lastLogTime = 0;
        uint32_t now = millis();
        if (now - lastLogTime > 100) {
            LOG_INFO(EventSource::AUTOSTEER,
                      "VWAS encoder: rawDelta=%d invertEncoder=%d deltaAngle=%.2f\u00B0 -> encoderAngle=%.2f\u00B0",
                      deltaPosition, config.invertEncoder ? 1 : 0, deltaAngle, encoderAngle);
            lastLogTime = now;
        }
    }
}

void WheelAngleFusion::updateGPSAngle() {
    if (!gnssProcessor) {
        gpsAngleValid = false;
        return;
    }

    const auto& gpsData = gnssProcessor->getData();

    // Vehicle speed - smoothed with the same 80% old / 20% new EMA that
    // AgOpenGPS itself applies to its VTG-derived speed. Our core distance-
    // based curvature calc doesn't actually divide by speed (that's the
    // whole point of the distance-gate rework), so this mainly benefits
    // the diagnostic display and the optional IMU-heading-rate path (which
    // still does divide by speed).
    if (gpsData.hasVelocity) {
        float rawSpeed = gpsData.speedKnots * 0.514444f;  // knots -> m/s
        vehicleSpeed = (vehicleSpeedInit) ? (0.8f * vehicleSpeed + 0.2f * rawSpeed) : rawSpeed;
        vehicleSpeedInit = true;
    } else {
        vehicleSpeed = 0.0f;
    }

    // --- Choose heading source ---
    // Prefer the UM982 dual-antenna heading (gpsData.dualHeading): a true
    // compass reading from the antenna baseline, unlike course-over-ground
    // (gpsData.headingTrue, from VTG) which requires the vehicle to be
    // moving to be accurate.
    bool haveGoodDualHeading = config.preferDualHeading &&
                                gpsData.hasDualHeading &&
                                gpsData.headingQuality >= config.minHeadingQuality;

    float currentHeading;
    if (haveGoodDualHeading) {
        currentHeading = gpsData.dualHeading;
        usingDualHeadingNow = true;
        lastHeadingQualityUsed = gpsData.headingQuality;
    } else {
        currentHeading = gpsData.headingTrue;
        usingDualHeadingNow = false;
        lastHeadingQualityUsed = 0;
    }

    uint32_t now = millis();

    // Diagnostic-only instantaneous heading rate (not used for the angle
    // calculation any more, only for logging / the optional IMU path).
    //if (lastGPSTime > 0 && now - lastGPSTime < 500) {
    //    float dtRate = (now - lastGPSTime) / 1000.0f;
    //    float headingDeltaForRate = currentHeading - lastHeading;
    //    if (headingDeltaForRate > 180.0f) headingDeltaForRate -= 360.0f;
    //    if (headingDeltaForRate < -180.0f) headingDeltaForRate += 360.0f;
    //    headingRate = (dtRate > 0.0f) ? (headingDeltaForRate / dtRate) : 0.0f;
    //}
    // FIX: Use smoothed GPS angle for rate calculation (avoid raw GPS noise)
    static float lastSmoothedForRate = 0.0f;
    static uint32_t lastRateCalcTime = 0;
    float dtRate = (lastRateCalcTime > 0) ? (now - lastRateCalcTime) / 1000.0f : 0.1f;
    if (dtRate > 0.001f && dtRate < 5.0f) {
        float hDelta = gpsAngleSmoothed - lastSmoothedForRate;
        hDelta = normalizeAngle(hDelta);
        headingRate = hDelta / dtRate;
        lastSmoothedForRate = gpsAngleSmoothed;
        lastRateCalcTime = now;
    }
    lastHeading = currentHeading;
    lastGPSTime = now;

    gpsAngle = 0.0f;  // Default: no correction this cycle
    bool haveHeadingDelta = false;
    float headingDelta = 0.0f;
    float distance = 0.0f;

    if (gpsData.hasPosition && gpsData.fixQuality > 0) {
        // --- Ring-buffer lookback ---
        // Instead of waiting forward for minDistanceForCurvature of new
        // travel (which at ~0.04 m/s takes ~2.5s per update), search
        // backward through recent history for the OLDEST sample that is
        // still >= minDistanceForCurvature away from the current position.
        // This gives a fresh candidate on every GPS fix, at any speed,
        // while still requiring a real, adequately-sized baseline.
        constexpr double metersPerDegreeLat = 111320.0;
        int16_t foundIdx = -1;
        float foundDistance = 0.0f;

        for (uint16_t i = 0; i < gpsHistoryCount; i++) {
            // Iterate oldest-first: index arithmetic on the circular buffer.
            uint16_t idx = (gpsHistoryHead + GPS_HISTORY_SIZE - gpsHistoryCount + i)
                           % GPS_HISTORY_SIZE;
//            double dNorth = (gpsData.latitude - gpsHistory[idx].lat) * metersPerDegreeLat;
//            double dEast = (gpsData.longitude - gpsHistory[idx].lon) * metersPerDegreeLat *
//                           cos(gpsData.latitude * (double)DEG_TO_RAD);
            // FIX: Clamp latitude for cos() to avoid NaN near poles
            double latClamped = fmax(-89.9, fmin(89.9, gpsData.latitude * DEG_TO_RAD));
            double dNorth = (gpsData.latitude - gpsHistory[idx].lat) * metersPerDegreeLat;
            double dEast = (gpsData.longitude - gpsHistory[idx].lon) * metersPerDegreeLat * cos(latClamped);
            float d = (float)sqrt(dNorth * dNorth + dEast * dEast);
            if (d >= config.minDistanceForCurvature) {
                foundIdx = (int16_t)idx;
                foundDistance = d;
                break;  // oldest qualifying sample - stop here (see design note below)
            }
        }

        if (foundIdx >= 0) {
            distance = foundDistance;
            lastDistanceTraveled = distance;

            // Plausibility check: reject implausibly large implied speed
            // (GPS jump / multipath glitch), rather than requiring a
            // minimum speed. Real slow-speed driving is never rejected by
            // this - only clearly-broken samples are.
            float timeSpan = (now - gpsHistory[foundIdx].timeMs) / 1000.0f;
            float impliedSpeed = (timeSpan > 0.001f) ? (distance / timeSpan) : 999.0f;

            if (impliedSpeed <= config.maxPlausibleSpeed) {
                headingDelta = currentHeading - gpsHistory[foundIdx].heading;
                if (headingDelta > 180.0f) headingDelta -= 360.0f;
                if (headingDelta < -180.0f) headingDelta += 360.0f;
                haveHeadingDelta = true;
            }
            // else: treat as a glitch, skip this cycle entirely (haveHeadingDelta
            // stays false) rather than feeding a corrupted sample downstream.
        }

        // Push the current sample into the ring buffer for future lookups.
        gpsHistory[gpsHistoryHead].lat = gpsData.latitude;
        gpsHistory[gpsHistoryHead].lon = gpsData.longitude;
        gpsHistory[gpsHistoryHead].heading = currentHeading;
        gpsHistory[gpsHistoryHead].timeMs = now;
        gpsHistoryHead = (gpsHistoryHead + 1) % GPS_HISTORY_SIZE;
        if (gpsHistoryCount < GPS_HISTORY_SIZE) gpsHistoryCount++;
    }

    // --- Vehicle-type-specific raw angle candidate ---
    bool haveCandidate = false;
    float rawAngleCandidate = 0.0f;

    if (config.vehicleType == 2) {
        // Articulated ("Knicklenker"): both antennas sit on the front
        // frame. A change in front-frame heading since a fixed reference
        // point approximates the articulation angle change since that
        // reference (rear-frame yaw rate is comparatively small at the
        // slow speeds this is designed for). Deliberately NOT an
        // incremental sum of per-cycle deltas: with the ring-buffer
        // lookback, consecutive lookback windows overlap heavily at low
        // speed, and summing overlapping deltas would count the same
        // physical rotation multiple times. Computing directly against a
        // single fixed reference avoids that entirely.
        if (haveHeadingDelta) {
            if (!haveReferenceHeadingArticulated) {
                referenceHeadingArticulated = currentHeading;
                haveReferenceHeadingArticulated = true;
                gpsIntegratedAngle = 0.0f;
            } else {
                float totalDelta = currentHeading - referenceHeadingArticulated;
                if (totalDelta > 180.0f) totalDelta -= 360.0f;
                if (totalDelta < -180.0f) totalDelta += 360.0f;
                gpsIntegratedAngle = totalDelta;
            }
            if (gpsIntegratedAngle > config.maxSteeringAngle) {
                gpsIntegratedAngle = config.maxSteeringAngle;
            } else if (gpsIntegratedAngle < -config.maxSteeringAngle) {
                gpsIntegratedAngle = -config.maxSteeringAngle;
            }
            rawAngleCandidate = gpsIntegratedAngle;
            haveCandidate = true;
        }
    } else {
        // Standard (Ackermann front-wheel steering, vehicleType 0 and
        // default fallback for any other/unhandled type).
        // curvature = headingDelta / distanceTravelled = 1/turnRadius.
        // Safe to recompute freely from overlapping lookback windows since
        // this feeds an EMA filter below, not a running sum.
        if (haveHeadingDelta) {
            float headingDeltaRad = headingDelta * DEG_TO_RAD;
            float curvature = headingDeltaRad / distance;  // 1/m
            float angleRad = atan(curvature * config.wheelbase);
            rawAngleCandidate = angleRad * RAD_TO_DEG;

        //    if (rawAngleCandidate > config.maxSteeringAngle) {
        //        rawAngleCandidate = config.maxSteeringAngle;
        //    } else if (rawAngleCandidate < -config.maxSteeringAngle) {
        //        rawAngleCandidate = -config.maxSteeringAngle;
        //    }
        // FIX: Plausibility reject instead of hard clip (40° saturation bug)
            if (abs(rawAngleCandidate) > config.maxSteeringAngle * 1.2f) {
                LOG_DEBUG(EventSource::AUTOSTEER,
                          "GPS candidate rejected: |%.2f°| > %.2f° threshold",
                          rawAngleCandidate, config.maxSteeringAngle * 1.2f);
                gpsAngleValid = false;
                return;
            }
            rawAngleCandidate = constrain(rawAngleCandidate, -config.maxSteeringAngle, config.maxSteeringAngle);
            haveCandidate = true;
        }

        // Optional IMU yaw-rate path (unchanged, still time-rate based -
        // only used if explicitly enabled).
        if (config.useIMUHeadingRate && imuProcessor) {
            const auto& imuData = imuProcessor->getCurrentData();
            if (imuData.isValid) {
                headingRate = imuData.yawRate;
                rawAngleCandidate = calculateGPSAngleFromHeadingRate(headingRate, vehicleSpeed);
                haveCandidate = (vehicleSpeed >= config.minSpeedForGPS);
            }
        }
    }

    if (!haveCandidate) {
        gpsAngleValid = false;
        return;
    }

    // --- EMA smoothing ---
    // A single moving-average/low-pass stage on the heading-derived angle
    // (rather than a second Kalman layer) - simpler to reason about and
    // tune, and avoids a Kalman-on-Kalman noise interaction.
    float dtSmooth = 0.1f;  // approx cycle time; fusion runs at 100Hz driven externally
    float alpha = dtSmooth / (config.gpsAngleSmoothingTau + dtSmooth);
    if (!gpsAngleSmoothedInit) {
        gpsAngleSmoothed = rawAngleCandidate;
        gpsAngleSmoothedInit = true;
    } else {
        gpsAngleSmoothed = gpsAngleSmoothed + alpha * (rawAngleCandidate - gpsAngleSmoothed);
    }
    gpsAngle = gpsAngleSmoothed;

    // Validity gate: the distance/plausibility gate above is now the real
    // protection - no additional minimum-speed requirement, so this works
    // correctly at your ~0.04 m/s work speed as well as at road-transport
    // speed.
    gpsAngleValid = haveCandidate;

    if (gpsAngleValid) {
        LOG_DEBUG(EventSource::AUTOSTEER,
                  "GPS angle: %.2f° (raw %.2f°, type=%d, source=%s q=%d, speed: %.2f m/s, dist: %.2fm)",
                  gpsAngle, rawAngleCandidate, config.vehicleType,
                  usingDualHeadingNow ? "dual" : "COG",
                  lastHeadingQualityUsed, vehicleSpeed, lastDistanceTraveled);
    }
}


void WheelAngleFusion::updateKalmanFilter(float dt) {
    // --- Encoder-dominant fusion ---
    // fusedAngle is ALWAYS the encoder-derived angle. GPS/dual-heading never
    // directly replaces or blends into the instantaneous angle - it only
    // ever nudges encoderOffset, slowly and rate-limited, in
    // updateDriftCompensation() below. This matches the proven approach
    // used by Cerea, AOG_Teensy_UM98X and 87yj/AgOpen_Keya_VirtualWAS:
    // "encoder dominates during rapid changes, heading only compensates
    // for drift" - rather than a symmetric Kalman blend that lets a single
    // noisy heading sample swing the reported angle by many degrees.
    predictedAngle = encoderAngle - encoderOffset;
    fusedAngle = predictedAngle;

    // uncertainty/kalmanGain are kept only as diagnostic/health indicators
    // now (see isHealthy()), not as active blend weights.
    uncertainty = min(uncertainty + config.processNoise * dt, config.initialUncertainty);
    kalmanGain = 0.0f;  // No longer used to blend fusedAngle; see updateDriftCompensation()

    // Constrain final angle to reasonable limits
    if (fusedAngle > config.maxSteeringAngle) {
        fusedAngle = config.maxSteeringAngle;
    } else if (fusedAngle < -config.maxSteeringAngle) {
        fusedAngle = -config.maxSteeringAngle;
    }

    if (uncertainty < 0.001f) {
        uncertainty = 0.001f;
    }
}

void WheelAngleFusion::updateVariance() {
    // Using fixed variance - adaptive calculation not yet implemented
    measurementVariance = 1.0f;
}

void WheelAngleFusion::updateDriftCompensation(float dt) {
    // Slowly trim encoderOffset toward agreement with the smoothed
    // GPS/dual-heading-derived angle. This never touches fusedAngle
    // directly (that stays encoder-only, see updateKalmanFilter) - it only
    // adjusts the long-term zero-point, and only by a small, rate-limited
    // amount per correction event.
    if (!gpsAngleValid) {
        driftRate = 0.0f;
        return;
    }

    // Rate-limit using the ACTUAL elapsed time since the last correction,
    // not the fusion loop's per-cycle dt. gpsAngleValid only pulses true
    // once per distance-gated GPS sample - at your ~0.04 m/s work speed
    // that's roughly every 2.5s, far longer than the 10ms fusion cycle.
    // Using the fixed 10ms dt here would make maxDriftCorrectionRate
    // apply ~250x too small a step per real correction event.
    uint32_t now = millis();
    float elapsed = (lastDriftCorrectionTime > 0) ?
                     (now - lastDriftCorrectionTime) / 1000.0f : dt;
    lastDriftCorrectionTime = now;
    // Guard against an implausibly long gap (e.g. after being stationary
    // for a while) turning into one huge correction step.
    if (elapsed > 5.0f) elapsed = 5.0f;

    float currentAngle = encoderAngle - encoderOffset;
    float innovation = gpsAngle - currentAngle;

    // Outlier rejection: a large disagreement most likely means a genuine
    // manoeuvre, bad GPS, or the encoder scale/sign being wrong - not slow
    // drift. Skip the correction entirely rather than partially applying
    // it (partial application of bad data was the original bug).
    if (abs(innovation) > config.driftInnovationRejectThreshold) {
        return;
    }

    // Proportional correction, hard-capped in degrees/second so GPS can
    // never yank the reported angle quickly regardless of tuning mistakes
    // elsewhere.
    float desiredCorrection = innovation;  // degrees we'd like to close
    float maxStep = config.maxDriftCorrectionRate * elapsed;
    float step = constrain(desiredCorrection * 0.1f, -maxStep, maxStep);

    // encoderOffset is defined such that fusedAngle = encoderAngle - encoderOffset.
    // To move currentAngle toward gpsAngle (i.e. increase currentAngle when
    // innovation > 0), we decrease encoderOffset by `step`.
    encoderOffset -= step;
    driftRate = step / elapsed;
    encoderDrift = encoderOffset;  // kept for external getters/telemetry
}

float WheelAngleFusion::calculateGPSAngleFromHeadingRate(float headingRate, float speed) {
    // Ackermann steering geometry
    // wheel_angle = atan(heading_rate * wheelbase / speed) * RAD_TO_DEG
    
    if (speed < config.minSpeedForGPS) {
        return 0.0f;
    }
    
    float headingRateRad = headingRate * DEG_TO_RAD;
    float angleRad = atan(headingRateRad * config.wheelbase / speed);
    float angleDeg = angleRad * RAD_TO_DEG;
    
    // Sanity check
    if (angleDeg > config.maxSteeringAngle) {
        angleDeg = config.maxSteeringAngle;
    } else if (angleDeg < -config.maxSteeringAngle) {
        angleDeg = -config.maxSteeringAngle;
    }
    
    return angleDeg;
}

bool WheelAngleFusion::isValidGPSConditions() const {
    // Check if conditions are suitable for GPS angle calculation
    return (vehicleSpeed >= config.minSpeedForGPS &&
            abs(headingRate) < config.maxHeadingRate &&
            abs(fusedAngle) < 30.0f);  // Relatively straight driving
}

bool WheelAngleFusion::isHealthy() const {
    // Check overall system health.
    //
    // IMPORTANT: this deliberately does NOT require GPS/heading-rate
    // validity or a minimum speed. GPS fusion only *improves* long-term
    // drift correction while driving in a straight-ish line; the
    // encoder-derived estimate (predictedAngle / fusedAngle in the
    // no-GPS branch of updateKalmanFilter) is still a meaningful angle
    // at standstill or at low speed. If this returned false whenever GPS
    // wasn't aiding, AutosteerProcessor falls back to the raw analog WAS
    // input - which, on a machine with NO physical WAS sensor wired up,
    // is a floating ADC pin producing noise. That fallback is strictly
    // worse than the encoder-only estimate, so we do not gate on it here.
    uint32_t now = millis();
    
    if (!keyaDriver) {
        return false;  // Never initialized / no position feedback
    }
    
    // Check if we're getting updates
    if (now - lastUpdateTime > 1000) {
        return false;  // No updates for 1 second
    }
    
    // Check if uncertainty is reasonable
    if (uncertainty > 50.0f) {
        return false;  // Too uncertain (e.g. long time without GPS aiding)
    }
    
    // Check if angle is reasonable
    if (abs(fusedAngle) > config.maxSteeringAngle * 1.5f) {
        return false;  // Angle out of bounds - something is very wrong
    }
    
    return true;
}

void WheelAngleFusion::startCalibration() {
    LOG_INFO(EventSource::AUTOSTEER, "Starting wheel angle fusion calibration");
    calibrationMode = true;
    calibrationMinAngle = 999.0f;
    calibrationMaxAngle = -999.0f;
    calibrationMinPosition = INT32_MAX;
    calibrationMaxPosition = INT32_MIN;
}

void WheelAngleFusion::stopCalibration() {
    if (!calibrationMode) return;
    
    calibrationMode = false;
    
    // Calculate counts per degree from calibration data
    if (calibrationMaxAngle > calibrationMinAngle && 
        calibrationMaxPosition > calibrationMinPosition) {
        
        float angleRange = calibrationMaxAngle - calibrationMinAngle;
        int32_t positionRange = calibrationMaxPosition - calibrationMinPosition;
        
        config.countsPerDegree = (float)positionRange / angleRange;
        
        LOG_INFO(EventSource::AUTOSTEER, "Calibration complete:");
        LOG_INFO(EventSource::AUTOSTEER, "  Angle range: %.1f° to %.1f°", 
                 calibrationMinAngle, calibrationMaxAngle);
        LOG_INFO(EventSource::AUTOSTEER, "  Position range: %d to %d", 
                 calibrationMinPosition, calibrationMaxPosition);
        LOG_INFO(EventSource::AUTOSTEER, "  Counts per degree: %.2f", config.countsPerDegree);
    } else {
        LOG_ERROR(EventSource::AUTOSTEER, "Calibration failed - insufficient data");
    }
}

void WheelAngleFusion::setEncoderCenter() {
    if (!keyaDriver) {
        LOG_ERROR(EventSource::AUTOSTEER, "Cannot set encoder center - no motor driver");
        return;
    }
    
    // Store current accumulated angle as the offset, then zero it out.
    // Call this with the wheels physically straight.
    encoderOffset = encoderAngle;  // Store current angle as offset
    encoderAngle = 0.0f;           // Reset to zero
    
    // Reset the delta tracking in the driver so the next getPositionDelta()
    // call doesn't report a large jump.
    keyaDriver->getPositionDelta();
    
    LOG_INFO(EventSource::AUTOSTEER, "Encoder center set (offset: %.2f°)", encoderOffset);
}

void WheelAngleFusion::reset() {
    LOG_INFO(EventSource::AUTOSTEER, "Resetting wheel angle fusion");
    
    // Reset Kalman filter
    fusedAngle = 0.0f;
    predictedAngle = 0.0f;
    uncertainty = config.initialUncertainty;
    kalmanGain = 0.0f;
    
    // Reset sensor angles
    encoderAngle = 0.0f;
    gpsAngle = 0.0f;
    gpsAngleValid = false;
    
    // Reset drift compensation
    encoderDrift = 0.0f;
    driftRate = 0.0f;
    driftStartTime = millis();

    // Reset distance-based curvature / integration / smoothing state
    gpsHistoryHead = 0;
    gpsHistoryCount = 0;
    lastDistanceTraveled = 0.0f;
    gpsAngleSmoothed = 0.0f;
    gpsAngleSmoothedInit = false;
    gpsIntegratedAngle = 0.0f;
    gpsIntegratedAngleInit = false;
    referenceHeadingArticulated = 0.0f;
    haveReferenceHeadingArticulated = false;
    vehicleSpeedInit = false;
    encoderOffset = 0.0f;
    lastDriftCorrectionTime = 0;
    
    // Clear variance buffer
    for (uint16_t i = 0; i < config.varianceBufferSize; i++) {
        varianceBuffer[i] = 0.0f;
    }
    varianceIndex = 0;
    varianceCount = 0;
    measurementVariance = 1.0f;
}

void WheelAngleFusion::resetDriftCompensation() {
    LOG_INFO(EventSource::AUTOSTEER, "Resetting drift compensation");
    encoderDrift = 0.0f;
    driftRate = 0.0f;
    driftStartTime = millis();
}