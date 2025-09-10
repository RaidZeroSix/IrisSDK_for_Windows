/**
    @file ShockTesterGUI.cpp
    @brief Shock Tester GUI Application for Iris Dynamics Orca Motor
    @version 1.0
    
    Application for running predefined shock test profiles on Orca motors.
    Loads force vs time profiles and executes them with simple operator controls.
*/

#define IRISCONTROLS

#include "ShockTesterGUI.h"
#include "ic4_library/iriscontrols4.h"
#include "modbus_client/device_applications/actuator.h"
#include "library_linker.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <windows.h>

using namespace std;

// Motor configuration - adjust COM port as needed
Actuator motor{90, "Shock Tester Motor", 1};

// GUI instance
ShockTesterGUI gui;
IrisControls4* IC4_virtual = &gui;

// Test control variables
bool test_running = false;
bool emergency_stop = false;
int current_repetition = 0;
int total_repetitions = 1;
string current_test_name = "";

// Force profile data
vector<float> profile_times_ms;
vector<float> profile_forces_N;
int profile_index = 0;
uint32_t test_start_time = 0;
uint32_t profile_start_time = 0;

// Connection and timing
bool was_connected = false;
unsigned int last_gui_update = 0;
const unsigned int GUI_UPDATE_PERIOD = 16; // ~60Hz GUI update

// Available test profiles
struct TestProfile {
    string name;
    string filename;
    string description;
    float peak_force_N;
    float duration_ms;
};

vector<TestProfile> available_tests = {
    {"Half Sine 10g", "profiles/half_sine_10g.csv", "10g half-sine shock, 11ms", 980.0f, 11.0f},
    {"Half Sine 50g", "profiles/half_sine_50g.csv", "50g half-sine shock, 6ms", 4900.0f, 6.0f},
    {"Sawtooth 30g", "profiles/sawtooth_30g.csv", "30g sawtooth shock, 8ms", 2940.0f, 8.0f},
    {"Square Wave 20g", "profiles/square_20g.csv", "20g square wave, 5ms", 1960.0f, 5.0f},
    {"Custom Profile", "profiles/custom.csv", "User-defined profile", 0.0f, 0.0f}
};

int selected_test_index = 0;

/**
 * Load force profile from CSV file
 * Expected format: time_ms,force_N per line
 */
bool loadForceProfile(const string& filename) {
    profile_times_ms.clear();
    profile_forces_N.clear();
    
    ifstream file(filename);
    if (!file.is_open()) {
        cout << "Error: Could not open profile file: " << filename << endl;
        return false;
    }
    
    string line;
    // Skip header if present
    getline(file, line);
    if (line.find("time") != string::npos || line.find("Time") != string::npos) {
        // Header detected, already skipped
    } else {
        // No header, process first line
        stringstream ss(line);
        float time, force;
        char comma;
        if (ss >> time >> comma >> force) {
            profile_times_ms.push_back(time);
            profile_forces_N.push_back(force);
        }
    }
    
    // Read data
    while (getline(file, line)) {
        stringstream ss(line);
        float time, force;
        char comma;
        if (ss >> time >> comma >> force) {
            profile_times_ms.push_back(time);
            profile_forces_N.push_back(force);
        }
    }
    
    file.close();
    
    if (profile_times_ms.empty()) {
        cout << "Error: No valid data found in profile file" << endl;
        return false;
    }
    
    cout << "Loaded profile with " << profile_times_ms.size() << " points" << endl;
    cout << "Duration: " << profile_times_ms.back() << " ms" << endl;
    cout << "Max force: " << *max_element(profile_forces_N.begin(), profile_forces_N.end()) << " N" << endl;
    
    return true;
}

/**
 * Get interpolated force value for current time in profile
 */
float getForceAtTime(float time_ms) {
    if (profile_times_ms.empty()) return 0.0f;
    
    // Before start
    if (time_ms <= profile_times_ms[0]) {
        return profile_forces_N[0];
    }
    
    // After end
    if (time_ms >= profile_times_ms.back()) {
        return 0.0f; // Return to zero after profile completes
    }
    
    // Find interpolation points
    for (size_t i = 1; i < profile_times_ms.size(); i++) {
        if (time_ms <= profile_times_ms[i]) {
            // Linear interpolation
            float t1 = profile_times_ms[i-1];
            float t2 = profile_times_ms[i];
            float f1 = profile_forces_N[i-1];
            float f2 = profile_forces_N[i];
            
            float ratio = (time_ms - t1) / (t2 - t1);
            return f1 + ratio * (f2 - f1);
        }
    }
    
    return 0.0f;
}

/**
 * Start a shock test
 */
void startTest() {
    if (test_running) return;
    
    // Load the selected profile
    TestProfile& test = available_tests[selected_test_index];
    if (!loadForceProfile(test.filename)) {
        cout << "Failed to load test profile!" << endl;
        return;
    }
    
    current_test_name = test.name;
    test_running = true;
    emergency_stop = false;
    current_repetition = 0;
    profile_index = 0;
    test_start_time = GetTickCount();
    
    // Switch motor to force mode
    motor.set_mode(Actuator::ForceMode);
    
    cout << "Starting test: " << current_test_name << endl;
    cout << "Repetitions: " << total_repetitions << endl;
}

/**
 * Stop the current test
 */
void stopTest() {
    test_running = false;
    emergency_stop = true;
    
    // Command zero force and switch to sleep mode
    motor.set_force_mN(0);
    motor.set_mode(Actuator::SleepMode);
    
    cout << "Test stopped!" << endl;
}

/**
 * Execute one repetition of the shock profile
 */
void runTestCycle() {
    if (!test_running || emergency_stop) return;
    
    uint32_t current_time = GetTickCount();
    
    // Check if starting new repetition
    if (profile_index == 0) {
        profile_start_time = current_time;
        current_repetition++;
        cout << "Running repetition " << current_repetition << " of " << total_repetitions << endl;
    }
    
    // Calculate time within current profile
    float profile_time = (float)(current_time - profile_start_time);
    
    // Get force command for current time
    float force_N = getForceAtTime(profile_time);
    int force_mN = (int)(force_N * 1000.0f);
    
    // Send force command to motor
    motor.set_force_mN(force_mN);
    
    // Check if profile is complete
    if (profile_time > profile_times_ms.back() + 100) { // 100ms settling time
        profile_index = 0;
        
        // Check if all repetitions complete
        if (current_repetition >= total_repetitions) {
            cout << "Test complete!" << endl;
            stopTest();
        } else {
            // Add delay between repetitions
            Sleep(500);
        }
    }
}

/**
 * Initialize motor on first connection
 */
void onMotorConnect() {
    cout << "Motor connected!" << endl;
    
    // Configure motor settings
    motor.set_mode(Actuator::SleepMode);
    
    // Set force limits for safety
    motor.set_force_limit(50000); // 50N max force - adjust as needed
    
    cout << "Motor initialized in sleep mode" << endl;
}

/**
 * Main program loop
 */
int main() {
    int ic_port_number = 0;
    int motor_port_number = 0;
    
    cout << "=== Shock Tester Control System ===" << endl;
    cout << endl;
    
    cout << "Enter the COM port number for the motor (RS422): ";
    cin >> motor_port_number;
    
    cout << "Enter the virtual COM port number for IrisControls: ";
    cin >> ic_port_number;
    
    // Initialize motor communication
    motor.set_new_comport(motor_port_number);
    motor.init();
    motor.enable();
    
    // Initialize GUI
    IC4_virtual->setup(ic_port_number);
    
    cout << endl;
    cout << "System ready. Use IrisControls interface to run tests." << endl;
    cout << "Press Ctrl+C to exit" << endl;
    cout << endl;
    
    while (1) {
        // Motor communication
        if (motor.is_connected()) {
            if (!was_connected) {
                onMotorConnect();
                was_connected = true;
            }
            
            // Run test if active
            if (test_running) {
                runTestCycle();
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