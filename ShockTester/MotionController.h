/**
 * @file MotionController.h
 * @brief Motion control system for shock tester with smart homing and force profiles
 */

#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "modbus_client/device_applications/actuator.h"
#include <string>
#include <windows.h>

// Motion states for shock testing
enum MotionState {
    STATE_IDLE,           // Waiting at zero position
    STATE_ACCELERATE,     // Applying positive force until critical_position_1
    STATE_BRAKE,          // Applying negative force until critical_position_2
    STATE_STABILIZE,      // PID control to stabilization position
    STATE_RETURN_HOME,    // Moving back to zero for next rep
    STATE_ERROR          // Error state
};

// Homing states for smart endstop detection
enum HomingState {
    HOMING_IDLE,
    HOMING_START,
    FINDING_FAR_ENDSTOP,   // Moving to zero position (furthest from motor)
    ZEROING,               // Setting zero at far endstop
    FINDING_NEAR_ENDSTOP,  // Moving to measure total travel
    MEASURING,             // Recording max position
    RETURNING_TO_ZERO,     // Going back to start position
    HOMING_COMPLETE,
    HOMING_FAILED
};

// Shock test profile structure
struct ShockProfile {
    std::string name;
    float acceleration_force_N;      // Force during acceleration phase
    float braking_force_N;          // Force during braking phase (negative)
    float critical_position_1_mm;   // Position to start braking
    float critical_position_2_mm;   // Position to end braking (from opposite direction)
    float stabilization_position_mm; // Position to hold after braking
    float stabilization_time_ms;    // Time to hold stable position
};

// Simple PID controller for stabilization
class PIDController {
private:
    float kp, ki, kd;
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
    
public:
    PIDController(float p = 5.0f, float i = 0.2f, float d = 1.0f) 
        : kp(p), ki(i), kd(d), integral(0), prev_error(0),
          integral_limit(100.0f), output_limit(200.0f) {}
    
    void reset() {
        integral = 0;
        prev_error = 0;
    }
    
    void setGains(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }
    
    float calculate(float current_pos, float target_pos, float dt) {
        float error = target_pos - current_pos;
        
        // Proportional
        float P = kp * error;
        
        // Integral with anti-windup
        integral += error * dt;
        if (integral > integral_limit) integral = integral_limit;
        if (integral < -integral_limit) integral = -integral_limit;
        float I = ki * integral;
        
        // Derivative
        float derivative = (error - prev_error) / dt;
        float D = kd * derivative;
        prev_error = error;
        
        // Total output with limiting
        float output = P + I + D;
        if (output > output_limit) output = output_limit;
        if (output < -output_limit) output = -output_limit;
        
        return output;
    }
};

class MotionController {
private:
    Actuator& motor;
    
    // State machines
    MotionState motion_state;
    HomingState homing_state;
    
    // Position tracking
    float current_position_mm;
    float current_velocity_mms;
    float previous_position_mm;
    float max_position_mm;  // Measured during homing
    float min_position_mm;  // Should be 0 after homing
    
    // Force tracking
    float current_force_N;
    float commanded_force_N;
    
    // Timing
    uint32_t state_start_time;
    uint32_t last_update_time;
    uint32_t stall_start_time;
    
    // Homing parameters
    const float HOMING_VELOCITY_MMS = 30.0f;
    const float HOMING_FORCE_N = 50.0f;
    const float STALL_VELOCITY_THRESHOLD_MMS = 5.0f;
    const uint32_t STALL_TIME_MS = 750;
    
    // Current profile and repetitions
    ShockProfile current_profile;
    int current_repetition;
    int total_repetitions;
    
    // PID for stabilization
    PIDController pid;
    
    // Flags
    bool is_homed;
    bool test_running;
    bool emergency_stop;
    
    // Direction tracking for critical position 2
    bool approaching_from_positive;
    
public:
    MotionController(Actuator& motor_ref);
    
    // Main update function (call at 1000 Hz)
    void update();
    
    // Homing functions
    void startHoming();
    bool isHomed() const { return is_homed; }
    HomingState getHomingState() const { return homing_state; }
    float getMaxTravel() const { return max_position_mm; }
    
    // Test control
    void startTest(const ShockProfile& profile, int repetitions);
    void stopTest();
    void emergencyStop();
    bool isTestRunning() const { return test_running; }
    
    // Status getters
    MotionState getMotionState() const { return motion_state; }
    float getPosition() const { return current_position_mm; }
    float getVelocity() const { return current_velocity_mms; }
    float getForce() const { return current_force_N; }
    int getCurrentRep() const { return current_repetition; }
    int getTotalReps() const { return total_repetitions; }
    std::string getStateString() const;
    std::string getHomingStateString() const;
    
    // Profile management
    void setProfile(const ShockProfile& profile) { current_profile = profile; }
    const ShockProfile& getProfile() const { return current_profile; }
    
private:
    // Update functions
    void updateMeasurements();
    void updateVelocity(float dt);
    
    // Homing logic
    void updateHoming();
    bool isAtEndstop();
    
    // Motion logic
    void updateMotion();
    void applyForce(float force_N);
    
    // State transition helpers
    void transitionToState(MotionState new_state);
    bool hasPassedCriticalPosition2();
};

#endif // MOTION_CONTROLLER_H