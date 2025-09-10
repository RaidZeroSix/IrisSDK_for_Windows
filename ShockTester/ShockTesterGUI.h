/**
    @file ShockTesterGUI.h
    @brief GUI interface header for Shock Tester Application
    @version 1.0
*/

#ifndef SHOCK_TESTER_GUI_H
#define SHOCK_TESTER_GUI_H

#include "ic4_library/iriscontrols4.h"
#include "modbus_client/device_applications/actuator.h"
#include <string>
#include <vector>

// Forward declarations
extern Actuator motor;
extern bool test_running;
extern bool emergency_stop;
extern int current_repetition;
extern int total_repetitions;
extern std::string current_test_name;
extern std::vector<float> profile_times_ms;
extern std::vector<float> profile_forces_N;
extern int selected_test_index;

struct TestProfile;
extern std::vector<TestProfile> available_tests;

// Function declarations
void startTest();
void stopTest();
bool loadForceProfile(const std::string& filename);

/**
 * @class ShockTesterGUI
 * @brief Custom GUI for shock tester control interface
 */
class ShockTesterGUI : public IrisControls4 {
private:
    // GUI Elements - Main Control Panel
    FlexButton start_button{"START_TEST", "Start Test", 200, 60};
    FlexButton stop_button{"STOP_TEST", "EMERGENCY STOP", 200, 60};
    FlexButton load_profile_button{"LOAD_PROFILE", "Load Custom", 150, 40};
    
    // Test Selection
    FlexDropdown test_selector{"TEST_SELECT", "Select Test Profile", 300};
    FlexLabel test_description{"TEST_DESC", "Test Description", 400};
    
    // Test Parameters
    FlexSlider repetition_slider{"REPETITIONS", "Number of Repetitions", 1, 100, 1, 250};
    FlexData repetition_display{"REP_DISPLAY", "Current Repetition", "0 / 0", 150};
    
    // Motor Status Display
    FlexLabel motor_status{"MOTOR_STATUS", "Motor Status", 200};
    FlexData position_display{"POSITION", "Position (mm)", "0.0", 100};
    FlexData force_display{"FORCE", "Force (N)", "0.0", 100};
    FlexData temperature_display{"TEMP", "Temperature (°C)", "0.0", 100};
    
    // Test Status
    FlexLabel test_status{"TEST_STATUS", "Test Status", 300};
    FlexData test_name_display{"TEST_NAME", "Current Test", "None", 200};
    FlexData test_progress{"TEST_PROGRESS", "Progress", "Idle", 150};
    
    // Force Profile Visualization
    FlexPlot force_plot{"FORCE_PLOT", "Force Profile", -100, 100, 400, 200};
    FlexPlot actual_force_plot{"ACTUAL_FORCE", "Actual Force", -100, 100, 400, 200};
    
    // Safety Indicators
    FlexLabel safety_status{"SAFETY", "Safety Status", 200};
    FlexButton reset_button{"RESET", "Reset System", 120, 40};
    
    // Settings Panel
    FlexSlider force_limit_slider{"FORCE_LIMIT", "Force Limit (N)", 10, 200, 50, 250};
    FlexSlider sample_rate_slider{"SAMPLE_RATE", "Sample Rate (Hz)", 100, 2000, 1000, 250};
    
public:
    /**
     * Initialize GUI elements when connected to IrisControls
     */
    void initiate() {
        // Main page setup
        add_page("main", "Shock Tester Control");
        
        // Control Section
        add_info_box("control_box", "Test Control", 0);
        add(start_button);
        add_to_info_box("control_box");
        start_button.set_color(0, 200, 0); // Green
        
        add(stop_button);
        add_to_info_box("control_box");
        stop_button.set_color(200, 0, 0); // Red
        
        // Test Selection Section
        add_info_box("test_select_box", "Test Selection", 1);
        
        // Populate test dropdown
        test_selector.clear();
        for (const auto& test : available_tests) {
            test_selector.add_option(test.name.c_str());
        }
        add(test_selector);
        add_to_info_box("test_select_box");
        
        add(test_description);
        add_to_info_box("test_select_box");
        
        add(repetition_slider);
        add_to_info_box("test_select_box");
        
        add(load_profile_button);
        add_to_info_box("test_select_box");
        
        // Status Display Section
        add_info_box("status_box", "System Status", 2);
        
        add(motor_status);
        add_to_info_box("status_box");
        
        add(position_display);
        add_to_info_box("status_box");
        
        add(force_display);
        add_to_info_box("status_box");
        
        add(temperature_display);
        add_to_info_box("status_box");
        
        add(repetition_display);
        add_to_info_box("status_box");
        
        // Test Progress Section
        add_info_box("progress_box", "Test Progress", 3);
        
        add(test_status);
        add_to_info_box("progress_box");
        
        add(test_name_display);
        add_to_info_box("progress_box");
        
        add(test_progress);
        add_to_info_box("progress_box");
        
        // Force Visualization
        add_info_box("plot_box", "Force Profiles", 4);
        
        add(force_plot);
        add_to_info_box("plot_box");
        force_plot.set_color(0, 0, 255); // Blue for target
        
        add(actual_force_plot);
        add_to_info_box("plot_box");
        actual_force_plot.set_color(255, 0, 0); // Red for actual
        
        // Safety Section
        add_info_box("safety_box", "Safety", 5);
        
        add(safety_status);
        add_to_info_box("safety_box");
        
        add(reset_button);
        add_to_info_box("safety_box");
        
        // Settings Page
        add_page("settings", "Settings");
        
        add_info_box("settings_box", "System Settings", 0);
        
        add(force_limit_slider);
        add_to_info_box("settings_box");
        
        add(sample_rate_slider);
        add_to_info_box("settings_box");
        
        // Set initial values
        updateStatus();
    }
    
    /**
     * Update GUI elements - called periodically
     */
    void run() {
        // Handle button presses
        if (start_button.pressed()) {
            if (!test_running) {
                total_repetitions = (int)repetition_slider.get_value();
                selected_test_index = test_selector.get_selected_index();
                startTest();
            }
        }
        
        if (stop_button.pressed() || reset_button.pressed()) {
            stopTest();
        }
        
        // Update test selection display
        if (test_selector.changed()) {
            selected_test_index = test_selector.get_selected_index();
            if (selected_test_index >= 0 && selected_test_index < available_tests.size()) {
                test_description.set(available_tests[selected_test_index].description.c_str());
            }
        }
        
        // Update status displays
        updateStatus();
        
        // Update plots if test is running
        if (test_running) {
            updatePlots();
        }
    }
    
private:
    /**
     * Update all status displays
     */
    void updateStatus() {
        // Motor status
        if (motor.is_connected()) {
            motor_status.set("Motor Connected");
            motor_status.set_color(0, 200, 0);
            
            // Update motor data
            char buffer[50];
            
            sprintf(buffer, "%.2f", motor.get_position_mm());
            position_display.set(buffer);
            
            sprintf(buffer, "%.2f", motor.get_force_mN() / 1000.0f);
            force_display.set(buffer);
            
            sprintf(buffer, "%.1f", motor.get_temperature());
            temperature_display.set(buffer);
        } else {
            motor_status.set("Motor Disconnected");
            motor_status.set_color(200, 0, 0);
        }
        
        // Test status
        if (test_running) {
            test_status.set("Test Running");
            test_status.set_color(0, 200, 0);
            test_progress.set("Active");
            
            char rep_buffer[50];
            sprintf(rep_buffer, "%d / %d", current_repetition, total_repetitions);
            repetition_display.set(rep_buffer);
            
            test_name_display.set(current_test_name.c_str());
            
            // Disable controls during test
            start_button.set_enabled(false);
            test_selector.set_enabled(false);
            repetition_slider.set_enabled(false);
        } else {
            test_status.set("Ready");
            test_status.set_color(100, 100, 100);
            test_progress.set("Idle");
            
            // Enable controls
            start_button.set_enabled(true);
            test_selector.set_enabled(true);
            repetition_slider.set_enabled(true);
        }
        
        // Safety status
        if (emergency_stop) {
            safety_status.set("EMERGENCY STOP ACTIVE");
            safety_status.set_color(200, 0, 0);
        } else {
            safety_status.set("System Safe");
            safety_status.set_color(0, 200, 0);
        }
    }
    
    /**
     * Update force profile plots
     */
    void updatePlots() {
        if (!profile_forces_N.empty()) {
            // Plot target profile (simplified - would need buffering for real implementation)
            static int plot_index = 0;
            if (plot_index < profile_forces_N.size()) {
                force_plot.add_sample(profile_forces_N[plot_index]);
                plot_index++;
            }
            
            // Plot actual force
            actual_force_plot.add_sample(motor.get_force_mN() / 1000.0f);
        }
    }
};

#endif // SHOCK_TESTER_GUI_H