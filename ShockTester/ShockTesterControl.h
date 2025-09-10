/**
    @file ShockTesterControl.h
    @brief Header file for Shock Tester Control System
    @version 2.0
*/

#ifndef SHOCK_TESTER_CONTROL_H
#define SHOCK_TESTER_CONTROL_H

#include "ic4_library/iriscontrols4.h"
#include "modbus_client/device_applications/actuator.h"
#include "ShockTestProfiles.h"
#include <string>
#include <vector>
#include <cmath>

// State machine states
enum TestState {
    IDLE,
    HOMING,
    ACCELERATE,
    BRAKE,
    SHOCK_DETECTED,
    STABILIZE,
    COMPLETE
};

// Forward declarations
extern Actuator motor;
extern TestState current_state;
extern bool test_running;
extern bool emergency_stop;
extern int current_repetition;
extern int total_repetitions;
extern ShockTestProfile current_profile;
extern std::vector<ShockTestProfile> test_profiles;
extern int selected_profile_index;

// Measurements
extern float current_position_mm;
extern float current_force_N;
extern float current_velocity_mms;
extern float max_force_recorded;

// Function declarations
void startTest();
void stopTest();
void updateStateMachine();
void applyForceForState();
bool velocityReversed();
bool positionSettled();
const char* getStateName(TestState state);

/**
 * @class SimplePID
 * @brief Simple PID controller for position stabilization
 */
class SimplePID {
private:
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
    
public:
    SimplePID() : Kp(10.0f), Ki(0.5f), Kd(2.0f), 
                  integral(0), prev_error(0), 
                  integral_limit(100.0f), output_limit(100.0f) {}
    
    void setGains(float p, float i, float d) {
        Kp = p;
        Ki = i;
        Kd = d;
    }
    
    void setLimits(float int_limit, float out_limit) {
        integral_limit = int_limit;
        output_limit = out_limit;
    }
    
    void reset() {
        integral = 0;
        prev_error = 0;
    }
    
    int calculate(float current_pos_mm, float target_pos_mm) {
        const float dt = 0.001f; // 1ms
        
        float error = target_pos_mm - current_pos_mm;
        
        // Proportional term
        float P = Kp * error;
        
        // Integral term with anti-windup
        integral += error * dt;
        if (integral > integral_limit) integral = integral_limit;
        if (integral < -integral_limit) integral = -integral_limit;
        float I = Ki * integral;
        
        // Derivative term
        float derivative = (error - prev_error) / dt;
        float D = Kd * derivative;
        prev_error = error;
        
        // Calculate total output
        float force_N = P + I + D;
        
        // Clamp output
        if (force_N > output_limit) force_N = output_limit;
        if (force_N < -output_limit) force_N = -output_limit;
        
        // Convert to milliNewtons
        return (int)(force_N * 1000.0f);
    }
};

extern SimplePID position_pid;

/**
 * @class ShockTesterGUI
 * @brief Custom GUI for shock tester control interface
 */
class ShockTesterGUI : public IrisControls4 {
private:
    // GUI Elements - Main Control Panel
    FlexButton start_button{"START_TEST", "Start Test", 200, 60};
    FlexButton stop_button{"STOP_TEST", "EMERGENCY STOP", 200, 60};
    
    // Test Selection
    FlexDropdown test_selector{"TEST_SELECT", "Select Test Profile", 300};
    FlexLabel test_description{"TEST_DESC", "Test Description", 400};
    
    // Test Parameters Display
    FlexData accel_force_display{"ACCEL_FORCE", "Accel Force (N)", "0", 120};
    FlexData brake_force_display{"BRAKE_FORCE", "Brake Force (N)", "0", 120};
    FlexData critical_pos_display{"CRIT_POS", "Critical Pos (mm)", "0", 120};
    
    // Custom Profile Editors (for custom profile only)
    FlexSlider accel_force_slider{"ACCEL_SLIDER", "Acceleration Force (N)", 10, 500, 100, 300};
    FlexSlider brake_force_slider{"BRAKE_SLIDER", "Braking Force (N)", -500, -10, -200, 300};
    FlexSlider critical_pos_slider{"CRIT_POS_SLIDER", "Critical Position (mm)", 5, 50, 25, 300};
    FlexSlider stabilize_pos_slider{"STAB_POS_SLIDER", "Stabilize Position (mm)", 0, 30, 10, 300};
    
    // Test Control
    FlexSlider repetition_slider{"REPETITIONS", "Number of Repetitions", 1, 100, 1, 250};
    FlexData repetition_display{"REP_DISPLAY", "Current Repetition", "0 / 0", 150};
    
    // Motor Status Display
    FlexLabel motor_status{"MOTOR_STATUS", "Motor Status", 200};
    FlexData position_display{"POSITION", "Position (mm)", "0.0", 100};
    FlexData force_display{"FORCE", "Force (N)", "0.0", 100};
    FlexData velocity_display{"VELOCITY", "Velocity (mm/s)", "0.0", 100};
    FlexData temperature_display{"TEMP", "Temperature (°C)", "0.0", 100};
    
    // State Machine Status
    FlexLabel state_display{"STATE", "Current State", 200};
    FlexData max_force_display{"MAX_FORCE", "Peak Force (N)", "0.0", 120};
    
    // Test Status
    FlexLabel test_status{"TEST_STATUS", "Test Status", 300};
    FlexData test_name_display{"TEST_NAME", "Current Test", "None", 200};
    
    // Real-time Plots
    FlexPlot position_plot{"POS_PLOT", "Position (mm)", -10, 60, 400, 200};
    FlexPlot force_plot{"FORCE_PLOT", "Force (N)", -600, 600, 400, 200};
    
    // Safety
    FlexLabel safety_status{"SAFETY", "Safety Status", 200};
    FlexButton reset_button{"RESET", "Reset System", 120, 40};
    
public:
    /**
     * Initialize GUI elements when connected to IrisControls
     */
    void initiate() {
        // Main page
        add_page("main", "Shock Tester Control");
        
        // Control Section
        add_info_box("control_box", "Test Control", 0);
        add(start_button);
        add_to_info_box("control_box");
        start_button.set_color(0, 200, 0);
        
        add(stop_button);
        add_to_info_box("control_box");
        stop_button.set_color(200, 0, 0);
        
        // Test Selection
        add_info_box("test_select_box", "Test Selection", 1);
        
        test_selector.clear();
        for (const auto& test : test_profiles) {
            test_selector.add_option(test.name.c_str());
        }
        add(test_selector);
        add_to_info_box("test_select_box");
        
        add(test_description);
        add_to_info_box("test_select_box");
        
        // Test Parameters Display
        add(accel_force_display);
        add_to_info_box("test_select_box");
        add(brake_force_display);
        add_to_info_box("test_select_box");
        add(critical_pos_display);
        add_to_info_box("test_select_box");
        
        add(repetition_slider);
        add_to_info_box("test_select_box");
        
        // Status Display
        add_info_box("status_box", "System Status", 2);
        
        add(motor_status);
        add_to_info_box("status_box");
        
        add(state_display);
        add_to_info_box("status_box");
        
        add(position_display);
        add_to_info_box("status_box");
        
        add(force_display);
        add_to_info_box("status_box");
        
        add(velocity_display);
        add_to_info_box("status_box");
        
        add(temperature_display);
        add_to_info_box("status_box");
        
        add(repetition_display);
        add_to_info_box("status_box");
        
        add(max_force_display);
        add_to_info_box("status_box");
        
        // Test Progress
        add_info_box("progress_box", "Test Progress", 3);
        
        add(test_status);
        add_to_info_box("progress_box");
        
        add(test_name_display);
        add_to_info_box("progress_box");
        
        // Plots
        add_info_box("plot_box", "Real-time Data", 4);
        
        add(position_plot);
        add_to_info_box("plot_box");
        position_plot.set_color(0, 0, 255);
        
        add(force_plot);
        add_to_info_box("plot_box");
        force_plot.set_color(255, 0, 0);
        
        // Safety
        add_info_box("safety_box", "Safety", 5);
        
        add(safety_status);
        add_to_info_box("safety_box");
        
        add(reset_button);
        add_to_info_box("safety_box");
        
        // Custom Profile Page
        add_page("custom", "Custom Profile");
        
        add_info_box("custom_box", "Custom Test Parameters", 0);
        
        add(accel_force_slider);
        add_to_info_box("custom_box");
        
        add(brake_force_slider);
        add_to_info_box("custom_box");
        
        add(critical_pos_slider);
        add_to_info_box("custom_box");
        
        add(stabilize_pos_slider);
        add_to_info_box("custom_box");
        
        updateStatus();
    }
    
    /**
     * Update GUI elements
     */
    void run() {
        // Handle buttons
        if (start_button.pressed()) {
            if (!test_running) {
                total_repetitions = (int)repetition_slider.get_value();
                selected_profile_index = test_selector.get_selected_index();
                
                // Update custom profile if selected
                if (selected_profile_index == test_profiles.size() - 1) {
                    test_profiles[selected_profile_index].acceleration_force_N = accel_force_slider.get_value();
                    test_profiles[selected_profile_index].braking_force_N = brake_force_slider.get_value();
                    test_profiles[selected_profile_index].critical_position_mm = critical_pos_slider.get_value();
                    test_profiles[selected_profile_index].stabilize_position_mm = stabilize_pos_slider.get_value();
                }
                
                startTest();
            }
        }
        
        if (stop_button.pressed() || reset_button.pressed()) {
            stopTest();
        }
        
        // Update test selection
        if (test_selector.changed()) {
            selected_profile_index = test_selector.get_selected_index();
            if (selected_profile_index >= 0 && selected_profile_index < test_profiles.size()) {
                ShockTestProfile& prof = test_profiles[selected_profile_index];
                test_description.set(prof.description.c_str());
                
                char buffer[50];
                sprintf(buffer, "%.1f", prof.acceleration_force_N);
                accel_force_display.set(buffer);
                sprintf(buffer, "%.1f", prof.braking_force_N);
                brake_force_display.set(buffer);
                sprintf(buffer, "%.1f", prof.critical_position_mm);
                critical_pos_display.set(buffer);
                
                // Update sliders if custom profile
                if (selected_profile_index == test_profiles.size() - 1) {
                    accel_force_slider.set_value(prof.acceleration_force_N);
                    brake_force_slider.set_value(prof.braking_force_N);
                    critical_pos_slider.set_value(prof.critical_position_mm);
                    stabilize_pos_slider.set_value(prof.stabilize_position_mm);
                }
            }
        }
        
        updateStatus();
        
        if (test_running) {
            updatePlots();
        }
    }
    
private:
    void updateStatus() {
        // Motor status
        if (motor.is_connected()) {
            motor_status.set("Motor Connected");
            motor_status.set_color(0, 200, 0);
            
            char buffer[50];
            
            sprintf(buffer, "%.2f", current_position_mm);
            position_display.set(buffer);
            
            sprintf(buffer, "%.2f", current_force_N);
            force_display.set(buffer);
            
            sprintf(buffer, "%.1f", current_velocity_mms);
            velocity_display.set(buffer);
            
            sprintf(buffer, "%.1f", motor.get_temperature());
            temperature_display.set(buffer);
            
            sprintf(buffer, "%.1f", max_force_recorded);
            max_force_display.set(buffer);
        } else {
            motor_status.set("Motor Disconnected");
            motor_status.set_color(200, 0, 0);
        }
        
        // State display
        state_display.set(getStateName(current_state));
        switch(current_state) {
            case IDLE:
                state_display.set_color(100, 100, 100);
                break;
            case HOMING:
                state_display.set_color(0, 100, 200);
                break;
            case ACCELERATE:
                state_display.set_color(0, 200, 0);
                break;
            case BRAKE:
                state_display.set_color(200, 200, 0);
                break;
            case SHOCK_DETECTED:
                state_display.set_color(255, 100, 0);
                break;
            case STABILIZE:
                state_display.set_color(0, 150, 150);
                break;
            case COMPLETE:
                state_display.set_color(0, 200, 100);
                break;
        }
        
        // Test status
        if (test_running) {
            test_status.set("Test Running");
            test_status.set_color(0, 200, 0);
            
            char rep_buffer[50];
            sprintf(rep_buffer, "%d / %d", current_repetition, total_repetitions);
            repetition_display.set(rep_buffer);
            
            test_name_display.set(current_profile.name.c_str());
            
            start_button.set_enabled(false);
            test_selector.set_enabled(false);
            repetition_slider.set_enabled(false);
        } else {
            test_status.set("Ready");
            test_status.set_color(100, 100, 100);
            
            start_button.set_enabled(true);
            test_selector.set_enabled(true);
            repetition_slider.set_enabled(true);
        }
        
        // Safety
        if (emergency_stop) {
            safety_status.set("EMERGENCY STOP");
            safety_status.set_color(200, 0, 0);
        } else {
            safety_status.set("System Safe");
            safety_status.set_color(0, 200, 0);
        }
    }
    
    void updatePlots() {
        position_plot.add_sample(current_position_mm);
        force_plot.add_sample(current_force_N);
    }
};

#endif // SHOCK_TESTER_CONTROL_H