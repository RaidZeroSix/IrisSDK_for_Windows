/**
    @file ShockTesterControl.cpp
    @brief Shock Tester Control System using Force-Based Trapezoidal Motion
    @version 2.0
    
    Application for generating controlled shocks through force commands.
    Uses a state machine to apply acceleration and braking forces based on position feedback.
*/

#define IRISCONTROLS

#include "ShockTesterControl.h"
#include "ShockTestProfiles.h"
#include "ic4_library/iriscontrols4.h"
#include "modbus_client/device_applications/actuator.h"
#include "library_linker.h"
#include <iostream>
#include <windows.h>
#include <cmath>

using namespace std;

// Motor configuration
Actuator motor{90, "Shock Tester Motor", 1};

// GUI instance
ShockTesterGUI gui;
IrisControls4* IC4_virtual = &gui;

// State machine
TestState current_state = IDLE;
TestState previous_state = IDLE;

// Test control
ShockTestProfile current_profile;
bool test_running = false;
bool emergency_stop = false;
int current_repetition = 0;
int total_repetitions = 1;

// Position and force tracking
float current_position_mm = 0.0f;
float previous_position_mm = 0.0f;
float current_force_N = 0.0f;
float current_velocity_mms = 0.0f;
float max_force_recorded = 0.0f;

// Timing
uint32_t state_start_time = 0;
uint32_t test_start_time = 0;
uint32_t shock_time = 0;
uint32_t last_update_time = 0;
const float dt = 0.001f; // 1ms update rate

// Connection status
bool was_connected = false;
unsigned int last_gui_update = 0;
const unsigned int GUI_UPDATE_PERIOD = 16;

// PID controller for stabilization
SimplePID position_pid;

// Available test profiles
vector<ShockTestProfile> test_profiles = {
    // Light shock - 10g equivalent
    {
        "10g Light Shock",
        "Light shock test for delicate equipment",
        50.0f,    // acceleration_force_N
        -100.0f,  // braking_force_N
        25.0f,    // critical_position_mm
        10.0f,    // stabilize_position_mm
        200.0f,   // shock_threshold_N
        10.0f,    // velocity_threshold_mms
        50.0f,    // max_position_mm
        5000.0f   // timeout_ms
    },
    // Medium shock - 30g equivalent
    {
        "30g Medium Shock",
        "Standard shock test",
        150.0f,   // acceleration_force_N
        -300.0f,  // braking_force_N
        30.0f,    // critical_position_mm
        10.0f,    // stabilize_position_mm
        600.0f,   // shock_threshold_N
        10.0f,    // velocity_threshold_mms
        50.0f,    // max_position_mm
        5000.0f   // timeout_ms
    },
    // Heavy shock - 50g equivalent
    {
        "50g Heavy Shock",
        "High-G shock test for robust equipment",
        250.0f,   // acceleration_force_N
        -500.0f,  // braking_force_N
        35.0f,    // critical_position_mm
        10.0f,    // stabilize_position_mm
        1000.0f,  // shock_threshold_N
        10.0f,    // velocity_threshold_mms
        50.0f,    // max_position_mm
        5000.0f   // timeout_ms
    },
    // Custom profile (user-editable)
    {
        "Custom Profile",
        "User-defined test parameters",
        100.0f,   // acceleration_force_N
        -200.0f,  // braking_force_N
        20.0f,    // critical_position_mm
        10.0f,    // stabilize_position_mm
        400.0f,   // shock_threshold_N
        10.0f,    // velocity_threshold_mms
        50.0f,    // max_position_mm
        5000.0f   // timeout_ms
    }
};

int selected_profile_index = 0;

/**
 * Calculate velocity from position changes
 */
void updateVelocity() {
    uint32_t current_time = GetTickCount();
    float delta_time = (current_time - last_update_time) / 1000.0f; // Convert to seconds
    
    if (delta_time > 0) {
        current_velocity_mms = (current_position_mm - previous_position_mm) / delta_time;
        previous_position_mm = current_position_mm;
        last_update_time = current_time;
    }
}

/**
 * Check if velocity has reversed (indicates collision)
 */
bool velocityReversed() {
    static float previous_velocity = 0;
    bool reversed = false;
    
    // Check for sign change with threshold
    if (previous_velocity > current_profile.velocity_threshold_mms && 
        current_velocity_mms < -current_profile.velocity_threshold_mms) {
        reversed = true;
    }
    
    previous_velocity = current_velocity_mms;
    return reversed;
}

/**
 * Check if position has settled near target
 */
bool positionSettled() {
    float error = abs(current_position_mm - current_profile.stabilize_position_mm);
    float velocity_magnitude = abs(current_velocity_mms);
    
    // Position within 1mm and velocity low
    return (error < 1.0f && velocity_magnitude < 5.0f);
}

/**
 * Apply force based on current state
 */
void applyForceForState() {
    int force_mN = 0;
    
    switch(current_state) {
        case HOMING:
            // Slow movement to find home position
            force_mN = -20000; // -20N to move to negative limit
            break;
            
        case ACCELERATE:
            // Apply acceleration force
            force_mN = current_profile.acceleration_force_N * 1000;
            break;
            
        case BRAKE:
            // Apply braking force
            force_mN = current_profile.braking_force_N * 1000;
            break;
            
        case STABILIZE:
            // PID control to stabilization position
            force_mN = position_pid.calculate(
                current_position_mm, 
                current_profile.stabilize_position_mm
            );
            break;
            
        case IDLE:
        case COMPLETE:
        case SHOCK_DETECTED:
        default:
            force_mN = 0;
            break;
    }
    
    // Apply force command
    motor.set_force_mN(force_mN);
}

/**
 * Update state machine based on sensor feedback
 */
void updateStateMachine() {
    uint32_t current_time = GetTickCount();
    
    // Check for emergency stop
    if (emergency_stop) {
        current_state = IDLE;
        motor.set_force_mN(0);
        motor.set_mode(Actuator::SleepMode);
        return;
    }
    
    // Check for timeout
    if (test_running && (current_time - test_start_time) > current_profile.timeout_ms) {
        cout << "Test timeout!" << endl;
        stopTest();
        return;
    }
    
    // State transitions
    switch(current_state) {
        case IDLE:
            // Waiting for test start
            break;
            
        case HOMING:
            // Check if we've reached home position (force limit or position limit)
            if (current_position_mm <= -45.0f || abs(current_force_N) > 50.0f) {
                motor.zero_position(); // Set this as zero
                current_state = ACCELERATE;
                state_start_time = current_time;
                cout << "Homed. Starting acceleration." << endl;
            }
            break;
            
        case ACCELERATE:
            // Check if reached critical position
            if (current_position_mm >= current_profile.critical_position_mm) {
                current_state = BRAKE;
                state_start_time = current_time;
                cout << "Critical position reached at " << current_position_mm << "mm, braking!" << endl;
            }
            // Safety check - max position
            if (current_position_mm >= current_profile.max_position_mm) {
                cout << "Max position exceeded!" << endl;
                stopTest();
            }
            break;
            
        case BRAKE:
            // Detect shock through force spike or velocity reversal
            if (abs(current_force_N) > current_profile.shock_threshold_N) {
                current_state = SHOCK_DETECTED;
                shock_time = current_time;
                max_force_recorded = abs(current_force_N);
                cout << "Shock detected! Peak force: " << max_force_recorded << "N" << endl;
            }
            else if (velocityReversed()) {
                current_state = SHOCK_DETECTED;
                shock_time = current_time;
                cout << "Velocity reversal detected!" << endl;
            }
            break;
            
        case SHOCK_DETECTED:
            // Brief delay then stabilize
            if ((current_time - shock_time) > 100) {
                current_state = STABILIZE;
                state_start_time = current_time;
                position_pid.reset();
                cout << "Stabilizing position..." << endl;
            }
            break;
            
        case STABILIZE:
            // Check if position has settled
            if (positionSettled() || (current_time - state_start_time) > 2000) {
                current_state = COMPLETE;
                state_start_time = current_time;
                cout << "Repetition " << current_repetition << " complete." << endl;
            }
            break;
            
        case COMPLETE:
            // Check if more repetitions needed
            if (current_repetition < total_repetitions) {
                // Wait a bit between repetitions
                if ((current_time - state_start_time) > 500) {
                    current_state = HOMING;
                    state_start_time = current_time;
                }
            } else {
                test_running = false;
                current_state = IDLE;
                cout << "Test complete! Max force recorded: " << max_force_recorded << "N" << endl;
            }
            break;
    }
    
    // Log state changes
    if (current_state != previous_state) {
        cout << "State: " << getStateName(current_state) << endl;
        previous_state = current_state;
    }
}

/**
 * Start a shock test
 */
void startTest() {
    if (test_running) return;
    
    // Load selected profile
    current_profile = test_profiles[selected_profile_index];
    
    // Initialize test
    test_running = true;
    emergency_stop = false;
    current_repetition = 0;
    max_force_recorded = 0.0f;
    test_start_time = GetTickCount();
    
    // Start with homing
    current_state = HOMING;
    previous_state = IDLE;
    
    // Switch motor to force mode
    motor.set_mode(Actuator::ForceMode);
    
    // Configure PID for this profile
    position_pid.setGains(10.0f, 0.5f, 2.0f); // Can be tuned per profile
    
    cout << "Starting test: " << current_profile.name << endl;
    cout << "Acceleration Force: " << current_profile.acceleration_force_N << "N" << endl;
    cout << "Braking Force: " << current_profile.braking_force_N << "N" << endl;
    cout << "Critical Position: " << current_profile.critical_position_mm << "mm" << endl;
    cout << "Repetitions: " << total_repetitions << endl;
}

/**
 * Stop the current test
 */
void stopTest() {
    test_running = false;
    emergency_stop = true;
    current_state = IDLE;
    
    // Stop motor
    motor.set_force_mN(0);
    motor.set_mode(Actuator::SleepMode);
    
    cout << "Test stopped!" << endl;
}

/**
 * Get state name for logging
 */
const char* getStateName(TestState state) {
    switch(state) {
        case IDLE: return "IDLE";
        case HOMING: return "HOMING";
        case ACCELERATE: return "ACCELERATE";
        case BRAKE: return "BRAKE";
        case SHOCK_DETECTED: return "SHOCK_DETECTED";
        case STABILIZE: return "STABILIZE";
        case COMPLETE: return "COMPLETE";
        default: return "UNKNOWN";
    }
}

/**
 * Initialize motor on first connection
 */
void onMotorConnect() {
    cout << "Motor connected!" << endl;
    
    // Configure motor
    motor.set_mode(Actuator::SleepMode);
    
    // Set safety limits
    motor.set_max_force(1000000); // 1000N max
    motor.set_force_limit(800000); // 800N safety limit
    
    cout << "Motor initialized" << endl;
}

/**
 * Main program loop
 */
int main() {
    int ic_port_number = 0;
    int motor_port_number = 0;
    
    cout << "=== Shock Tester Control System v2.0 ===" << endl;
    cout << "Force-Based Trapezoidal Motion Control" << endl;
    cout << endl;
    
    cout << "Enter the COM port number for the motor (RS422): ";
    cin >> motor_port_number;
    
    cout << "Enter the virtual COM port number for IrisControls: ";
    cin >> ic_port_number;
    
    // Configure high-speed communication
    Actuator::ConnectionConfig connection_config;
    connection_config.target_baud_rate_bps = 1250000;
    connection_config.target_delay_us = 0;
    
    // Initialize motor
    motor.set_connection_config(connection_config);
    motor.set_new_comport(motor_port_number);
    motor.init();
    motor.enable();
    
    // Initialize GUI
    IC4_virtual->setup(ic_port_number);
    
    cout << endl;
    cout << "System ready. Use IrisControls to configure and run tests." << endl;
    cout << "Press Ctrl+C to exit" << endl;
    cout << endl;
    
    while (1) {
        // Update measurements
        current_position_mm = motor.get_position_um() / 1000.0f;
        current_force_N = motor.get_force_mN() / 1000.0f;
        updateVelocity();
        
        // Motor communication
        if (motor.is_connected()) {
            if (!was_connected) {
                onMotorConnect();
                was_connected = true;
            }
            
            // Run state machine
            if (test_running) {
                updateStateMachine();
                applyForceForState();
                
                // Increment repetition counter
                if (current_state == ACCELERATE && previous_state == HOMING) {
                    current_repetition++;
                }
            }
        } else {
            was_connected = false;
        }
        
        motor.run_in();
        motor.run_out();
        
        // GUI updates
        gui.check();
        gui.send();
        
        if (gui.new_connection()) {
            gui.initiate();
        }
        
        if (gui.is_connected() && (millis() - last_gui_update > GUI_UPDATE_PERIOD)) {
            last_gui_update = millis();
            gui.run();
            gui.end_of_frame();
        }
    }
    
    return 0;
}