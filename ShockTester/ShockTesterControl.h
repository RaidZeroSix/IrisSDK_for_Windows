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
#include <map>
#include <windows.h>

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
class ShockTesterGUI {
private:
    // Pages
    GUI_Page main_page;
    GUI_Page custom_page;

    // GUI Elements - Main Control Panel
    FlexButton start_button;
    FlexButton stop_button;
    
    // Test Selection
    FlexDropdown test_selector;
    std::vector<MenuOption> test_options;               // persistent options

    // Map MenuOption id -> profile index
    std::map<unsigned int, int> option_to_index;

    FlexLabel test_description;
    
    // Test Parameters Display
    FlexData accel_force_display;
    FlexData brake_force_display;
    FlexData critical_pos_display;
    
    // Custom Profile Editors (for custom profile only)
    FlexSlider accel_force_slider;
    FlexSlider brake_force_slider;
    FlexSlider critical_pos_slider;
    FlexSlider stabilize_pos_slider;
    
    // Test Control
    FlexSlider repetition_slider;
    FlexData repetition_display;
    
    // Motor Status Display
    FlexLabel motor_status;
    FlexData position_display;
    FlexData force_display;
    FlexData velocity_display;
    FlexData temperature_display;
    
    // State Machine Status
    FlexLabel state_display;
    FlexData max_force_display;
    
    // Test Status
    FlexLabel test_status;
    FlexLabel test_name_display;
    
    // Real-time Plots
    FlexPlot position_plot;
    FlexPlot force_plot;
    Dataset position_dataset;
    Dataset force_dataset;
    
    // Safety
    FlexLabel safety_status;
    FlexButton reset_button;
    
public:
    /**
     * Initialize GUI elements when connected to IrisControls
     */
    void initiate() {
        // Add pages
        main_page.add();
        custom_page.add();
        
        // Control Section (top-left)
        start_button.add(&main_page, "Start Test", -1, 0, 0, 2, 4);
        stop_button.add(&main_page, "EMERGENCY STOP", -1, 0, 4, 2, 4);
        
        // Test Selection
        test_selector.add(&main_page, 2, 0, 2, 8, 0);
        
        // Populate dropdown options
        test_options.resize(test_profiles.size());
        option_to_index.clear();
        for (size_t i = 0; i < test_profiles.size(); ++i) {
            test_selector.add_option(&test_options[i], test_profiles[i].name.c_str());
            option_to_index[(unsigned int)test_options[i].id()] = static_cast<int>(i);
        }
        
        test_description.add(&main_page, "Test Description:", 4, 0, 1, 8, 0);
        
        // Test Parameters Display (cast args to disambiguate)
        accel_force_display.add(&main_page, "Accel Force (N)", 5, 0, 1, 2, (int)0, (u16)1, "N", (u32)FlexData::UNITS);
        brake_force_display.add(&main_page, "Brake Force (N)", 5, 2, 1, 2, (int)0, (u16)1, "N", (u32)FlexData::UNITS);
        critical_pos_display.add(&main_page, "Critical Pos (mm)", 5, 4, 1, 2, (int)0, (u16)1, "mm", (u32)FlexData::UNITS);
        
        // Repetitions
        repetition_slider.add(&main_page, "Repetitions", 6, 0, 1, 4, 1, 100, 1, (u16)1, "reps", FlexSlider::ALLOW_INPUT);
        repetition_display.add(&main_page, "Current Repetition", 6, 4, 1, 2, (int)0, (u16)1, "reps", (u32)0);
        
        // Status Display
        motor_status.add(&main_page, "Motor: Disconnected", 7, 0, 1, 4, 0);
        state_display.add(&main_page, "State: IDLE", 7, 4, 1, 4, 0);
        
        position_display.add(&main_page, "Position (mm)", 8, 0, 1, 2, (int)0, (u16)1, "mm", (u32)FlexData::UNITS);
        force_display.add(&main_page, "Force (N)", 8, 2, 1, 2, (int)0, (u16)1, "N", (u32)FlexData::UNITS);
        velocity_display.add(&main_page, "Velocity (mm/s)", 8, 4, 1, 2, (int)0, (u16)1, "mm/s", (u32)FlexData::UNITS);
        temperature_display.add(&main_page, "Temperature (C)", 8, 6, 1, 2, (int)0, (u16)1, "C", (u32)FlexData::UNITS);
        
        max_force_display.add(&main_page, "Peak Force (N)", 9, 0, 1, 2, (int)0, (u16)1, "N", (u32)FlexData::UNITS);
        
        // Test Progress
        test_status.add(&main_page, "Status: Ready", 10, 0, 1, 4, 0);
        test_name_display.add(&main_page, "Current Test: None", 10, 4, 1, 4, 0);
        
        // Plots
        position_plot.add(&main_page, "Position (mm)", 11, 0, 4, 8, -10.0f, 60.0f, 0);
        force_plot.add(&main_page, "Force (N)", 15, 0, 4, 8, -600.0f, 600.0f, 0);
        
        // Datasets for plots
        position_dataset.add(&position_plot, "Position", "t", "mm", 0);
        force_dataset.add(&force_plot, "Force", "t", "N", 0);
        
        // Safety
        safety_status.add(&main_page, "Safety: OK", 19, 0, 1, 4, 0);
        reset_button.add(&main_page, "Reset System", -1, 19, 4, 1, 2);
        
        // Custom Profile Page elements
        accel_force_slider.add(&custom_page, "Acceleration Force (N)", 0, 0, 1, 6, 10, 500, 100, (u16)1, "N", FlexSlider::ALLOW_INPUT);
        brake_force_slider.add(&custom_page, "Braking Force (N)", 1, 0, 1, 6, -500, -10, -200, (u16)1, "N", FlexSlider::ALLOW_INPUT);
        critical_pos_slider.add(&custom_page, "Critical Position (mm)", 2, 0, 1, 6, 5, 50, 25, (u16)1, "mm", FlexSlider::ALLOW_INPUT);
        stabilize_pos_slider.add(&custom_page, "Stabilize Position (mm)", 3, 0, 1, 6, 0, 30, 10, (u16)1, "mm", FlexSlider::ALLOW_INPUT);
    }
    
    /**
     * Update GUI elements
     */
    void run() {
        // Handle buttons
        if (start_button.pressed()) {
            if (!test_running) {
                total_repetitions = (int)repetition_slider.get();
                // Selected profile from dropdown
                int opt_id = test_selector.get();
                if (option_to_index.count((unsigned int)opt_id)) {
                    selected_profile_index = option_to_index[(unsigned int)opt_id];
                }
                
                // Update custom profile if selected
                if (selected_profile_index == (int)test_profiles.size() - 1) {
                    test_profiles[selected_profile_index].acceleration_force_N = accel_force_slider.get_f();
                    test_profiles[selected_profile_index].braking_force_N = brake_force_slider.get_f();
                    test_profiles[selected_profile_index].critical_position_mm = critical_pos_slider.get_f();
                    test_profiles[selected_profile_index].stabilize_position_mm = stabilize_pos_slider.get_f();
                }
                
                startTest();
            }
        }
        
        if (stop_button.pressed() || reset_button.pressed()) {
            stopTest();
        }
        
        // Update test selection on change
        if (test_selector.new_value_received()) {
            int opt_id = test_selector.get();
            if (option_to_index.count((unsigned int)opt_id)) {
                selected_profile_index = option_to_index[(unsigned int)opt_id];
                if (selected_profile_index >= 0 && selected_profile_index < (int)test_profiles.size()) {
                    ShockTestProfile& prof = test_profiles[selected_profile_index];
                    test_description.rename(prof.description.c_str());
                    
                    accel_force_display.update(prof.acceleration_force_N);
                    brake_force_display.update(prof.braking_force_N);
                    critical_pos_display.update(prof.critical_position_mm);
                    
                    // Update sliders if custom profile
                    if (selected_profile_index == (int)test_profiles.size() - 1) {
                        accel_force_slider.update(prof.acceleration_force_N);
                        brake_force_slider.update(prof.braking_force_N);
                        critical_pos_slider.update(prof.critical_position_mm);
                        stabilize_pos_slider.update(prof.stabilize_position_mm);
                    }
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
            motor_status.rename("Motor: Connected");
            
            position_display.update(current_position_mm);
            force_display.update(current_force_N);
            velocity_display.update(current_velocity_mms);
            temperature_display.update(motor.get_temperature_C());
            
            max_force_display.update(max_force_recorded);
        } else {
            motor_status.rename("Motor: Disconnected");
        }
        
        // State display
        {
            std::string s = std::string("State: ") + getStateName(current_state);
            state_display.rename(s.c_str());
        }
        
        // Test status
        if (test_running) {
            test_status.rename("Status: Test Running");
            
            // display repetitions as number using the label next to slider
            repetition_display.update(current_repetition);
            
            {
                std::string tn = std::string("Current Test: ") + current_profile.name;
                test_name_display.rename(tn.c_str());
            }
            
            start_button.disable(true);
            test_selector.disable(true);
            repetition_slider.disable(true);
        } else {
            test_status.rename("Status: Ready");
            
            start_button.disable(false);
            test_selector.disable(false);
            repetition_slider.disable(false);
        }
        
        // Safety
        if (emergency_stop) {
            safety_status.rename("Safety: EMERGENCY STOP");
        } else {
            safety_status.rename("Safety: OK");
        }
    }
    
    void updatePlots() {
        // add latest samples using IC4 timebase in milliseconds
        float t_ms = (float)(IC4_virtual->system_time() / 1000.0);
        position_dataset.add_data(t_ms, current_position_mm);
        force_dataset.add_data(t_ms, current_force_N);
    }
};

/**
 * @class IC4WindowsTransport
 * @brief Minimal IrisControls4 transport implementation for Windows host
 */
class IC4WindowsTransport : public IrisControls4 {
    char print_buffer[MAX_VAR_STR_LENGTH];
public:
    void send() override {
        // Stub: transport handled by host application; nothing to flush here.
    }
    u64 system_time() override {
        LARGE_INTEGER freq, ticks;
        if (!QueryPerformanceFrequency(&freq)) {
            return (u64)GetTickCount64() * 1000ULL;
        }
        QueryPerformanceCounter(&ticks);
        return (u64)((ticks.QuadPart * 1000000ULL) / (u64)freq.QuadPart);
    }
    void handle_eot() override {}
#ifdef WINDOWS
    void setup(int comport) override {
        (void)comport;
        setup_sucess = true;
    }
#endif
    const char* val_to_str(int d) override {
        snprintf(print_buffer, sizeof(print_buffer), "%d", d);
        return print_buffer;
    }
    const char* val_to_str(unsigned int d) override {
        snprintf(print_buffer, sizeof(print_buffer), "%u", d);
        return print_buffer;
    }
    const char* val_to_str(u64 d) override {
        snprintf(print_buffer, sizeof(print_buffer), "%llu", (unsigned long long)d);
        return print_buffer;
    }
    const char* val_to_str(float f) override {
        snprintf(print_buffer, sizeof(print_buffer), "%g", f);
        return print_buffer;
    }
};

#endif // SHOCK_TESTER_CONTROL_H