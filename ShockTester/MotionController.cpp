/**
 * @file MotionController.cpp
 * @brief Implementation of motion control system for shock tester
 */

#include "MotionController.h"
#include <iostream>
#include <cmath>

using namespace std;

MotionController::MotionController(Actuator& motor_ref) 
    : motor(motor_ref),
      motion_state(STATE_IDLE),
      homing_state(HOMING_IDLE),
      current_position_mm(0),
      current_velocity_mms(0),
      previous_position_mm(0),
      max_position_mm(139.0f), // Default, will be measured
      min_position_mm(0),
      current_force_N(0),
      commanded_force_N(0),
      state_start_time(0),
      last_update_time(0),
      stall_start_time(0),
      current_repetition(0),
      total_repetitions(0),
      is_homed(false),
      test_running(false),
      emergency_stop(false),
      approaching_from_positive(false) {
}

void MotionController::update() {
    if (!motor.is_connected()) {
        if (test_running) {
            cout << "ERROR: Lost motor connection during test!" << endl;
            stopTest();
        }
        return;
    }
    
    // Update measurements from motor
    updateMeasurements();
    
    // Calculate time delta
    uint32_t now = GetTickCount();
    float dt = (now - last_update_time) / 1000.0f;
    if (dt > 0) {
        updateVelocity(dt);
    }
    last_update_time = now;
    
    // Handle emergency stop
    if (emergency_stop) {
        applyForce(0);
        return;
    }
    
    // Update appropriate state machine
    if (homing_state != HOMING_IDLE && homing_state != HOMING_COMPLETE) {
        updateHoming();
    } else if (test_running) {
        updateMotion();
    } else {
        // Idle - no force
        applyForce(0);
    }
}

void MotionController::updateMeasurements() {
    current_position_mm = motor.get_position_um() / 1000.0f;
    current_force_N = motor.get_force_mN() / 1000.0f;
}

void MotionController::updateVelocity(float dt) {
    current_velocity_mms = (current_position_mm - previous_position_mm) / dt;
    previous_position_mm = current_position_mm;
}

void MotionController::startHoming() {
    cout << "Starting homing sequence..." << endl;
    
    // Clear emergency stop if it was set
    if (emergency_stop) {
        cout << "Clearing emergency stop for homing..." << endl;
        emergency_stop = false;
    }
    
    // Ensure motor is enabled and in force mode
    if (motor.get_mode() == Actuator::SleepMode) {
        cout << "Waking motor from sleep mode..." << endl;
    }
    motor.enable();
    motor.set_mode(Actuator::ForceMode);
    cout << "Motor enabled in ForceMode for homing" << endl;
    
    homing_state = HOMING_START;
    is_homed = false;
    state_start_time = GetTickCount();
    stall_start_time = 0;
}

void MotionController::updateHoming() {
    uint32_t now = GetTickCount();
    
    switch (homing_state) {
        case HOMING_START:
            cout << "Moving to far endstop (zero position)..." << endl;
            homing_state = FINDING_FAR_ENDSTOP;
            state_start_time = now;
            break;
            
        case FINDING_FAR_ENDSTOP:
            // Apply negative force to move away from motor
            applyForce(-HOMING_FORCE_N);
            
            // Wait 1 second before checking for endstop to allow movement to start
            if (now - state_start_time > 1000) {
                if (isAtEndstop()) {
                    cout << "Found far endstop at position: " << current_position_mm << " mm" << endl;
                    homing_state = ZEROING;
                    state_start_time = now;
                }
            }
            
            // Timeout after 10 seconds
            if (now - state_start_time > 10000) {
                cout << "ERROR: Homing timeout - could not find far endstop" << endl;
                homing_state = HOMING_FAILED;
            }
            break;
            
        case ZEROING:
            // Stop force and zero position
            applyForce(0);
            motor.zero_position();
            min_position_mm = 0;
            cout << "Position zeroed at far endstop" << endl;
            
            // Wait a moment for position to update
            if (now - state_start_time > 500) {
                homing_state = FINDING_NEAR_ENDSTOP;
                state_start_time = now;
                cout << "Moving to near endstop to measure travel..." << endl;
            }
            break;
            
        case FINDING_NEAR_ENDSTOP:
            // Apply positive force to move toward motor
            applyForce(HOMING_FORCE_N);
            
            // Wait 1 second before checking for endstop to allow movement to start
            if (now - state_start_time > 1000) {
                if (isAtEndstop()) {
                    cout << "Found near endstop at position: " << current_position_mm << " mm" << endl;
                    homing_state = MEASURING;
                    state_start_time = now;
                }
            }
            
            // Timeout after 10 seconds
            if (now - state_start_time > 10000) {
                cout << "ERROR: Homing timeout - could not find near endstop" << endl;
                homing_state = HOMING_FAILED;
            }
            break;
            
        case MEASURING:
            // Stop force and record max position
            applyForce(0);
            max_position_mm = current_position_mm;
            cout << "Total travel measured: " << max_position_mm << " mm" << endl;
            
            // Wait a moment
            if (now - state_start_time > 500) {
                homing_state = RETURNING_TO_ZERO;
                state_start_time = now;
                cout << "Returning to zero position..." << endl;
            }
            break;
            
        case RETURNING_TO_ZERO:
            // Move back to zero position
            if (current_position_mm > 2.0f) {
                applyForce(-HOMING_FORCE_N);
            } else {
                // Close enough to zero
                applyForce(0);
                homing_state = HOMING_COMPLETE;
                is_homed = true;
                cout << "Homing complete! System ready at zero position." << endl;
                cout << "Travel range: 0 to " << max_position_mm << " mm" << endl;
            }
            
            // Timeout
            if (now - state_start_time > 10000) {
                cout << "ERROR: Could not return to zero position" << endl;
                homing_state = HOMING_FAILED;
            }
            break;
            
        case HOMING_FAILED:
            applyForce(0);
            is_homed = false;
            break;
            
        default:
            break;
    }
}

bool MotionController::isAtEndstop() {
    // Check if we're applying force but not moving (stalled)
    bool is_stalled = (abs(current_velocity_mms) < STALL_VELOCITY_THRESHOLD_MMS) &&
                      (abs(commanded_force_N) > 20.0f);
    
    // Debug output
    static int debug_counter = 0;
    if (++debug_counter % 50 == 0) {  // Every 50 calls (about 0.5 seconds)
        cout << "Endstop check - Vel: " << current_velocity_mms 
             << " mm/s, Cmd Force: " << commanded_force_N 
             << " N, Stalled: " << is_stalled << endl;
    }
    
    if (is_stalled) {
        if (stall_start_time == 0) {
            stall_start_time = GetTickCount();
            cout << "Stall detected, starting timer..." << endl;
        }
        
        // Check if stalled for long enough
        if (GetTickCount() - stall_start_time > STALL_TIME_MS) {
            cout << "Endstop reached after " << STALL_TIME_MS << " ms stall" << endl;
            stall_start_time = 0; // Reset for next time
            return true;
        }
    } else {
        stall_start_time = 0; // Reset if not stalled
    }
    
    return false;
}

void MotionController::startTest(const ShockProfile& profile, int repetitions) {
    // Clear emergency stop flag if it was set
    if (emergency_stop) {
        cout << "Clearing emergency stop state..." << endl;
        emergency_stop = false;
        
        // Wake motor from sleep if needed
        if (motor.get_mode() == Actuator::SleepMode) {
            cout << "Waking motor from sleep mode..." << endl;
            motor.enable();
            motor.set_mode(Actuator::ForceMode);
        }
    }
    
    if (!is_homed) {
        cout << "ERROR: System must be homed before starting test!" << endl;
        return;
    }
    
    current_profile = profile;
    total_repetitions = repetitions;
    current_repetition = 0;
    test_running = true;
    emergency_stop = false;
    
    // Start with homing to ensure we're at zero
    cout << "Starting test: " << profile.name << endl;
    cout << "Repetitions: " << repetitions << endl;
    
    // Begin homing sequence for first rep
    startHoming();
}

void MotionController::stopTest() {
    test_running = false;
    motion_state = STATE_IDLE;
    applyForce(0);
    // Don't put motor to sleep - keep it ready
    cout << "Test stopped." << endl;
}

void MotionController::emergencyStop() {
    emergency_stop = true;
    test_running = false;
    motion_state = STATE_IDLE;
    homing_state = HOMING_IDLE;
    applyForce(0);
    // For emergency stop, do put motor to sleep for safety
    motor.set_mode(Actuator::SleepMode);
    cout << "EMERGENCY STOP ACTIVATED!" << endl;
}

void MotionController::updateMotion() {
    uint32_t now = GetTickCount();
    
    // Check if homing is needed for this rep
    if (homing_state == HOMING_COMPLETE && motion_state == STATE_IDLE) {
        // Ready to start motion sequence
        current_repetition++;
        cout << "Starting repetition " << current_repetition << " of " << total_repetitions << endl;
        transitionToState(STATE_ACCELERATE);
        homing_state = HOMING_IDLE; // Reset homing state
    }
    
    switch (motion_state) {
        case STATE_IDLE:
            applyForce(0);
            break;
            
        case STATE_ACCELERATE:
            // Apply acceleration force
            applyForce(current_profile.acceleration_force_N);
            
            // Check if reached critical position 1
            if (current_position_mm >= current_profile.critical_position_1_mm) {
                cout << "Reached critical position 1 at " << current_position_mm << " mm, starting brake" << endl;
                transitionToState(STATE_BRAKE);
                approaching_from_positive = true; // We're approaching critical_position_2 from the positive side
            }
            break;
            
        case STATE_BRAKE:
            // Apply braking force
            applyForce(current_profile.braking_force_N);
            
            // Check if crossed critical position 2 from the correct direction
            if (hasPassedCriticalPosition2()) {
                cout << "Crossed critical position 2 at " << current_position_mm << " mm, stabilizing" << endl;
                transitionToState(STATE_STABILIZE);
                pid.reset();
            }
            break;
            
        case STATE_STABILIZE:
            // PID control to stabilization position
            {
                float force = pid.calculate(current_position_mm, 
                                           current_profile.stabilization_position_mm, 
                                           0.001f);
                applyForce(force);
            }
            
            // Hold for stabilization time
            if (now - state_start_time > current_profile.stabilization_time_ms) {
                cout << "Stabilization complete" << endl;
                transitionToState(STATE_RETURN_HOME);
            }
            break;
            
        case STATE_RETURN_HOME:
            // Check if more reps needed
            if (current_repetition < total_repetitions) {
                // Start homing for next rep
                startHoming();
                motion_state = STATE_IDLE;
            } else {
                // Test complete
                cout << "All repetitions complete!" << endl;
                test_running = false;
                motion_state = STATE_IDLE;
                applyForce(0);
            }
            break;
            
        case STATE_ERROR:
            applyForce(0);
            test_running = false;
            break;
    }
}

bool MotionController::hasPassedCriticalPosition2() {
    // We need to cross critical_position_2 from the positive side (moving negative)
    // This happens after braking from the acceleration phase
    if (approaching_from_positive) {
        return current_position_mm <= current_profile.critical_position_2_mm;
    }
    return false;
}

void MotionController::transitionToState(MotionState new_state) {
    motion_state = new_state;
    state_start_time = GetTickCount();
}

void MotionController::applyForce(float force_N) {
    commanded_force_N = force_N;
    
    // Ensure motor is awake and in force mode
    Actuator::MotorMode current_mode = motor.get_mode();
    if (current_mode == Actuator::SleepMode || current_mode != Actuator::ForceMode) {
        cout << "Current mode: " << current_mode << ", switching to ForceMode..." << endl;
        motor.enable();  // Wake up the motor
        motor.set_mode(Actuator::ForceMode);
        
        // Verify mode switch
        Sleep(10);  // Give time for mode switch
        Actuator::MotorMode new_mode = motor.get_mode();
        cout << "New mode: " << new_mode << " (ForceMode = " << Actuator::ForceMode << ")" << endl;
    }
    
    int force_mN = (int)(force_N * 1000.0f);
    motor.set_force_mN(force_mN);
    
    // Debug output
    static int debug_counter = 0;
    if (++debug_counter % 10 == 0) {  // Every 10 calls
        float actual_force = motor.get_force_mN() / 1000.0f;
        cout << "Commanded: " << force_N << " N, Actual: " << actual_force << " N" << endl;
    }
}

string MotionController::getStateString() const {
    switch (motion_state) {
        case STATE_IDLE: return "IDLE";
        case STATE_ACCELERATE: return "ACCELERATING";
        case STATE_BRAKE: return "BRAKING";
        case STATE_STABILIZE: return "STABILIZING";
        case STATE_RETURN_HOME: return "RETURNING HOME";
        case STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

string MotionController::getHomingStateString() const {
    switch (homing_state) {
        case HOMING_IDLE: return "IDLE";
        case HOMING_START: return "STARTING";
        case FINDING_FAR_ENDSTOP: return "FINDING FAR ENDSTOP";
        case ZEROING: return "ZEROING";
        case FINDING_NEAR_ENDSTOP: return "FINDING NEAR ENDSTOP";
        case MEASURING: return "MEASURING TRAVEL";
        case RETURNING_TO_ZERO: return "RETURNING TO ZERO";
        case HOMING_COMPLETE: return "COMPLETE";
        case HOMING_FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}