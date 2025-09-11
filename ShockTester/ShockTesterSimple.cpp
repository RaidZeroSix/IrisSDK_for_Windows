/**
    @file ShockTesterSimple.cpp
    @brief Simple Shock Tester - Console Application
    @version 3.0
    
    Simplified shock tester that actually works without external dependencies.
    Pure console application with clear status and controls.
*/

#include "modbus_client/device_applications/actuator.h"
#include "library_linker.h"
#include <iostream>
#include <windows.h>
#include <conio.h>
#include <cmath>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <string>

using namespace std;

// Motor configuration
Actuator motor{90, "Shock Motor", 1};

// Test states
enum State {
    IDLE,
    HOMING,
    ACCELERATE,
    BRAKE,
    SHOCK_DETECTED,
    STABILIZE,
    COMPLETE
};

// Test profile
struct Profile {
    string name;
    float accel_force_N;
    float brake_force_N;
    float critical_pos_mm;
    float stabilize_pos_mm;
    float shock_threshold_N;
};

// Available profiles
Profile profiles[] = {
    {"Light (10g)",  50.0f,  -100.0f, 25.0f, 10.0f, 200.0f},
    {"Medium (30g)", 150.0f, -300.0f, 30.0f, 10.0f, 600.0f},
    {"Heavy (50g)",  250.0f, -500.0f, 35.0f, 10.0f, 1000.0f},
    {"Custom",       100.0f, -200.0f, 20.0f, 10.0f, 400.0f}
};

// Global state
State current_state = IDLE;
Profile current_profile;
bool test_running = false;
bool emergency_stop = false;
int current_rep = 0;
int total_reps = 1;

// Measurements
float position_mm = 0;
float velocity_mms = 0;
float force_N = 0;
float prev_position = 0;
uint32_t prev_time = 0;

// Simple PID for stabilization
class PID {
    float kp = 10.0f, ki = 0.5f, kd = 2.0f;
    float integral = 0, prev_error = 0;
public:
    void reset() { integral = 0; prev_error = 0; }
    
    int calculate(float current, float target) {
        float error = target - current;
        integral += error * 0.001f;
        float derivative = (error - prev_error) / 0.001f;
        prev_error = error;
        
        float output = kp * error + ki * integral + kd * derivative;
        
        // Clamp to ±100N
        if (output > 100) output = 100;
        if (output < -100) output = -100;
        
        return (int)(output * 1000); // Convert to mN
    }
};

PID pid;

// Clear console line
void clearLine() {
    cout << "\r" << string(80, ' ') << "\r";
}

// Print header
void printHeader() {
    system("cls");
    cout << "=================================================\n";
    cout << "       SHOCK TESTER CONTROL - SIMPLIFIED\n";
    cout << "=================================================\n\n";
}

// Test motor connection with detailed diagnostics
bool testMotorConnection(int port) {
    cout << "\n=== MOTOR CONNECTION TEST ===" << endl;
    cout << "Attempting connection on COM" << port << "..." << endl;
    
    // Step 1: Test if port can be opened
    char portName[20];
    sprintf(portName, "\\\\.\\COM%d", port);
    HANDLE testHandle = CreateFileA(portName,
                                   GENERIC_READ | GENERIC_WRITE,
                                   0, NULL, OPEN_EXISTING, 0, NULL);
    
    if (testHandle == INVALID_HANDLE_VALUE) {
        DWORD error = GetLastError();
        cout << "\n*** FAILED TO OPEN COM PORT ***" << endl;
        if (error == ERROR_ACCESS_DENIED) {
            cout << "Reason: Port is already in use by another program" << endl;
            cout << "Solution: Close any other programs using COM" << port << endl;
        } else if (error == ERROR_FILE_NOT_FOUND) {
            cout << "Reason: COM" << port << " does not exist" << endl;
            cout << "Solution: Check Device Manager for correct port number" << endl;
        } else {
            cout << "Reason: Windows error code " << error << endl;
            cout << "Solution: Check if RS422 adapter drivers are installed" << endl;
        }
        return false;
    }
    CloseHandle(testHandle);
    cout << "  ✓ Port opened successfully" << endl;
    
    // Step 2: Configure connection
    Actuator::ConnectionConfig config;
    config.target_baud_rate_bps = 1250000;
    config.target_delay_us = 0;
    
    motor.set_connection_config(config);
    motor.set_new_comport(port);
    
    // Step 3: Initialize MODBUS
    cout << "Initializing MODBUS communication..." << endl;
    motor.init();
    cout << "  ✓ MODBUS initialized" << endl;
    
    // Step 4: Enable high-speed communication
    cout << "Enabling high-speed streaming..." << endl;
    motor.enable();
    
    // Step 5: Wait for motor response
    cout << "Waiting for motor response";
    int attempts = 0;
    const int max_attempts = 50; // 5 seconds at 100ms intervals
    
    while (!motor.is_connected() && attempts < max_attempts) {
        cout << ".";
        Sleep(100);
        motor.run_in();
        motor.run_out();
        attempts++;
    }
    cout << endl;
    
    if (!motor.is_connected()) {
        cout << "\n*** MOTOR CONNECTION FAILED ***" << endl;
        cout << "Reason: No valid MODBUS response from motor after " << (max_attempts * 100 / 1000) << " seconds" << endl;
        cout << "\nPossible causes:" << endl;
        cout << "  1. Motor not powered on" << endl;
        cout << "  2. RS422 cable not connected to motor" << endl;
        cout << "  3. Wrong COM port (connected to different device)" << endl;
        cout << "  4. RS422 A/B lines reversed" << endl;
        cout << "  5. Baud rate mismatch (motor expects 1.25 Mbps)" << endl;
        cout << "  6. Broken cable or bad connection" << endl;
        
        cout << "\nTroubleshooting steps:" << endl;
        cout << "  - Verify motor power LED is on" << endl;
        cout << "  - Check RS422 connections at both ends" << endl;
        cout << "  - Try swapping A/B lines if using terminal blocks" << endl;
        cout << "  - Confirm this is the RS422 adapter port, not a regular COM port" << endl;
        
        return false;
    }
    
    cout << "\n*** CONNECTION SUCCESSFUL ***" << endl;
    
    // Get motor info to verify communication
    cout << "\nVerifying communication..." << endl;
    
    // Try to read some basic registers
    motor.get_position_um();
    motor.get_force_mN();
    motor.get_temperature();
    
    // Give it a moment to populate
    Sleep(100);
    motor.run_in();
    motor.run_out();
    
    // Display motor status
    cout << "\nMotor Status:" << endl;
    cout << "  Position: " << motor.get_position_um() / 1000.0f << " mm" << endl;
    cout << "  Force: " << motor.get_force_mN() / 1000.0f << " N" << endl;
    cout << "  Temperature: " << motor.get_temperature() << " C" << endl;
    
    // Test mode switching
    cout << "\nTesting mode control..." << endl;
    motor.set_mode(Actuator::SleepMode);
    Sleep(100);
    motor.run_in();
    motor.run_out();
    
    cout << "  Sleep mode set: OK" << endl;
    
    motor.set_mode(Actuator::ForceMode);
    Sleep(100);
    motor.run_in();
    motor.run_out();
    
    cout << "  Force mode set: OK" << endl;
    
    // Apply small test force
    cout << "\nApplying test force (5N for 0.5 seconds)..." << endl;
    motor.set_force_mN(5000);
    
    for (int i = 0; i < 5; i++) {
        Sleep(100);
        motor.run_in();
        motor.run_out();
        float pos = motor.get_position_um() / 1000.0f;
        cout << "  Position: " << pos << " mm" << endl;
    }
    
    // Stop force
    motor.set_force_mN(0);
    motor.set_mode(Actuator::SleepMode);
    motor.run_in();
    motor.run_out();
    
    // Set safety limits
    cout << "\nConfiguring safety limits..." << endl;
    motor.set_max_force(1000000); // 1000N max
    cout << "  Max force set to 1000N" << endl;
    
    cout << "\n=== CONNECTION TEST COMPLETE ===" << endl;
    cout << "Motor is ready for operation." << endl;
    
    return true;
}

// Print status
void printStatus() {
    // Move cursor to status area (line 6)
    COORD coord = {0, 6};
    SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
    
    cout << "STATE: ";
    switch(current_state) {
        case IDLE:           cout << "IDLE      "; break;
        case HOMING:         cout << "HOMING    "; break;
        case ACCELERATE:     cout << "ACCELERATE"; break;
        case BRAKE:          cout << "BRAKE     "; break;
        case SHOCK_DETECTED: cout << "SHOCK!    "; break;
        case STABILIZE:      cout << "STABILIZE "; break;
        case COMPLETE:       cout << "COMPLETE  "; break;
    }
    
    cout << " | POS: " << fixed << setprecision(1) << setw(6) << position_mm << " mm";
    cout << " | VEL: " << setw(6) << velocity_mms << " mm/s";
    cout << " | FORCE: " << setw(7) << force_N << " N  \n";
    
    if (test_running) {
        cout << "REP: " << current_rep << "/" << total_reps;
        cout << " | Profile: " << current_profile.name << "    \n";
    } else {
        cout << "                                             \n";
    }
    
    cout << "\nCONTROLS: [S]tart  [X]Stop  [Q]uit          \n";
    
    if (emergency_stop) {
        cout << "\n*** EMERGENCY STOP ACTIVE ***\n";
    }
    
    // Connection status
    if (!motor.is_connected()) {
        cout << "\n*** WARNING: MOTOR DISCONNECTED ***\n";
    }
}

// Update velocity
void updateVelocity() {
    uint32_t now = GetTickCount();
    float dt = (now - prev_time) / 1000.0f;
    if (dt > 0) {
        velocity_mms = (position_mm - prev_position) / dt;
        prev_position = position_mm;
        prev_time = now;
    }
}

// Apply force based on state
void applyForce() {
    int force_mN = 0;
    
    switch(current_state) {
        case HOMING:
            force_mN = -20000; // -20N to home
            break;
        case ACCELERATE:
            force_mN = current_profile.accel_force_N * 1000;
            break;
        case BRAKE:
            force_mN = current_profile.brake_force_N * 1000;
            break;
        case STABILIZE:
            force_mN = pid.calculate(position_mm, current_profile.stabilize_pos_mm);
            break;
        default:
            force_mN = 0;
    }
    
    motor.set_force_mN(force_mN);
}

// State machine update
void updateStateMachine() {
    static uint32_t state_timer = 0;
    uint32_t now = GetTickCount();
    
    switch(current_state) {
        case IDLE:
            // Waiting
            break;
            
        case HOMING:
            // Home until we hit limit or timeout
            if (position_mm <= -40 || (now - state_timer) > 3000) {
                motor.zero_position();
                current_state = ACCELERATE;
                state_timer = now;
                cout << "\n>>> Starting acceleration\n";
            }
            break;
            
        case ACCELERATE:
            // Accelerate until critical position
            if (position_mm >= current_profile.critical_pos_mm) {
                current_state = BRAKE;
                state_timer = now;
                cout << "\n>>> Braking at " << position_mm << "mm\n";
            }
            break;
            
        case BRAKE:
            // Brake until shock detected (force spike or velocity reversal)
            if (abs(force_N) > current_profile.shock_threshold_N) {
                current_state = SHOCK_DETECTED;
                state_timer = now;
                cout << "\n>>> SHOCK! Force: " << force_N << "N\n";
            } else if (velocity_mms < -10 && prev_position > position_mm) {
                current_state = SHOCK_DETECTED;
                state_timer = now;
                cout << "\n>>> SHOCK! Velocity reversed\n";
            }
            break;
            
        case SHOCK_DETECTED:
            // Brief pause then stabilize
            if (now - state_timer > 100) {
                current_state = STABILIZE;
                state_timer = now;
                pid.reset();
            }
            break;
            
        case STABILIZE:
            // Stabilize for 1 second
            if (now - state_timer > 1000) {
                current_state = COMPLETE;
                state_timer = now;
                cout << "\n>>> Rep " << current_rep << " complete\n";
            }
            break;
            
        case COMPLETE:
            // Check if more reps needed
            if (current_rep < total_reps) {
                current_state = HOMING;
                state_timer = now;
            } else {
                test_running = false;
                current_state = IDLE;
                cout << "\n>>> Test complete!\n";
            }
            break;
    }
}

// Start test
void startTest() {
    if (test_running) return;
    
    // Check connection first
    if (!motor.is_connected()) {
        cout << "\n*** ERROR: Motor not connected! ***\n";
        return;
    }
    
    test_running = true;
    emergency_stop = false;
    current_rep = 0;
    current_state = HOMING;
    
    motor.set_mode(Actuator::ForceMode);
    
    cout << "\n>>> Test started: " << current_profile.name << "\n";
    cout << ">>> Accel: " << current_profile.accel_force_N << "N";
    cout << " | Brake: " << current_profile.brake_force_N << "N";
    cout << " | Critical: " << current_profile.critical_pos_mm << "mm\n";
}

// Stop test
void stopTest() {
    test_running = false;
    emergency_stop = true;
    current_state = IDLE;
    
    motor.set_force_mN(0);
    motor.set_mode(Actuator::SleepMode);
    
    cout << "\n>>> Test stopped!\n";
}

// Scan for available COM ports using Windows registry
vector<int> getAvailableCOMPorts() {
    vector<int> ports;
    
    cout << "Scanning for available COM ports..." << endl;
    
    // Method 1: Query Windows registry for all COM ports
    HKEY hKey;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &hKey) == ERROR_SUCCESS) {
        char valueName[256];
        BYTE valueData[256];
        DWORD valueNameSize, valueDataSize, valueType;
        DWORD index = 0;
        
        // Enumerate all values in the registry key
        while (true) {
            valueNameSize = sizeof(valueName);
            valueDataSize = sizeof(valueData);
            
            if (RegEnumValueA(hKey, index++, valueName, &valueNameSize, NULL, &valueType, valueData, &valueDataSize) != ERROR_SUCCESS) {
                break;
            }
            
            if (valueType == REG_SZ) {
                // Extract COM port number from string like "COM46"
                string portStr((char*)valueData);
                if (portStr.substr(0, 3) == "COM") {
                    int portNum = atoi(portStr.substr(3).c_str());
                    if (portNum > 0) {
                        ports.push_back(portNum);
                    }
                }
            }
        }
        RegCloseKey(hKey);
    }
    
    // If registry method failed or found nothing, fall back to scanning
    if (ports.empty()) {
        cout << "Registry scan failed, trying direct port scan..." << endl;
        
        // Test COM1 through COM256 (maximum Windows allows)
        char portName[20];
        for (int i = 1; i <= 256; i++) {
            sprintf(portName, "\\\\.\\COM%d", i);
            
            HANDLE hPort = CreateFileA(portName,
                                      GENERIC_READ | GENERIC_WRITE,
                                      0,
                                      NULL,
                                      OPEN_EXISTING,
                                      0,
                                      NULL);
            
            if (hPort != INVALID_HANDLE_VALUE) {
                ports.push_back(i);
                CloseHandle(hPort);
            } else {
                DWORD error = GetLastError();
                // ERROR_ACCESS_DENIED means port exists but is in use
                if (error == ERROR_ACCESS_DENIED) {
                    ports.push_back(i);
                }
            }
        }
    }
    
    // Sort ports for better display
    sort(ports.begin(), ports.end());
    
    return ports;
}

// Main
int main() {
    printHeader();
    
    // Scan for available COM ports
    vector<int> available_ports = getAvailableCOMPorts();
    
    if (available_ports.empty()) {
        cout << "\n*** ERROR: No COM ports found! ***" << endl;
        cout << "Please check:" << endl;
        cout << "  1. RS422 adapter is connected" << endl;
        cout << "  2. Drivers are installed" << endl;
        cout << "  3. Device Manager shows COM ports" << endl;
        cout << "\nPress any key to exit...";
        _getch();
        return 1;
    }
    
    cout << "\nAvailable COM ports:" << endl;
    for (size_t i = 0; i < available_ports.size(); i++) {
        cout << "  [" << (i + 1) << "] COM" << available_ports[i];
        
        // Try to identify if it's in use
        char portName[20];
        sprintf(portName, "\\\\.\\COM%d", available_ports[i]);
        HANDLE hPort = CreateFileA(portName,
                                  GENERIC_READ | GENERIC_WRITE,
                                  0, NULL, OPEN_EXISTING, 0, NULL);
        if (hPort == INVALID_HANDLE_VALUE && GetLastError() == ERROR_ACCESS_DENIED) {
            cout << " (in use)";
        } else if (hPort != INVALID_HANDLE_VALUE) {
            CloseHandle(hPort);
        }
        cout << endl;
    }
    cout << "  [0] Exit" << endl;
    
    // Get motor port selection
    int motor_port = 0;
    bool connected = false;
    
    while (!connected) {
        cout << "\nSelect COM port for motor (1-" << available_ports.size() << ", or 0 to exit): ";
        int selection;
        cin >> selection;
        
        if (selection == 0) {
            cout << "Exiting..." << endl;
            return 0;
        }
        
        if (selection < 1 || selection > (int)available_ports.size()) {
            cout << "Invalid selection. Please try again." << endl;
            continue;
        }
        
        motor_port = available_ports[selection - 1];
        cout << "\nTrying COM" << motor_port << "..." << endl;
        
        connected = testMotorConnection(motor_port);
        
        if (!connected) {
            cout << "\nWould you like to try a different port? (y/n): ";
            char retry;
            cin >> retry;
            if (retry != 'y' && retry != 'Y') {
                cout << "Exiting..." << endl;
                return 1;
            }
        }
    }
    
    cout << "\nPress any key to continue to profile selection...";
    _getch();
    
    printHeader();
    
    // Select profile
    cout << "\nProfiles:\n";
    for (int i = 0; i < 4; i++) {
        cout << "  [" << i << "] " << profiles[i].name << "\n";
    }
    cout << "Select profile (0-3): ";
    int profile_idx;
    cin >> profile_idx;
    if (profile_idx < 0 || profile_idx > 3) profile_idx = 0;
    current_profile = profiles[profile_idx];
    
    // Custom profile?
    if (profile_idx == 3) {
        cout << "Accel force (N): ";
        cin >> current_profile.accel_force_N;
        cout << "Brake force (N, negative): ";
        cin >> current_profile.brake_force_N;
        cout << "Critical position (mm): ";
        cin >> current_profile.critical_pos_mm;
    }
    
    // Get repetitions
    cout << "Number of repetitions: ";
    cin >> total_reps;
    
    printHeader();
    cout << "Ready. Press 'S' to start.\n";
    
    // Main loop
    while (true) {
        // Check connection
        if (!motor.is_connected()) {
            if (test_running) {
                stopTest();
                cout << "\n*** ERROR: Lost connection to motor! ***\n";
            }
        } else {
            // Update measurements
            position_mm = motor.get_position_um() / 1000.0f;
            force_N = motor.get_force_mN() / 1000.0f;
            updateVelocity();
            
            // Update state machine
            if (test_running && !emergency_stop) {
                updateStateMachine();
                applyForce();
                
                // Track repetitions
                if (current_state == ACCELERATE && current_rep == 0) {
                    current_rep = 1;
                } else if (current_state == HOMING && current_rep > 0 && current_rep < total_reps) {
                    current_rep++;
                }
            }
        }
        
        // Handle keyboard
        if (_kbhit()) {
            char key = _getch();
            if (key == 's' || key == 'S') {
                if (!test_running) startTest();
            } else if (key == 'x' || key == 'X') {
                stopTest();
            } else if (key == 'q' || key == 'Q') {
                stopTest();
                break;
            }
        }
        
        // Update display
        static uint32_t last_display = 0;
        if (GetTickCount() - last_display > 100) {
            last_display = GetTickCount();
            printStatus();
        }
        
        // Motor communication
        motor.run_in();
        motor.run_out();
        
        Sleep(1); // 1ms loop
    }
    
    cout << "\nShutting down...\n";
    motor.set_force_mN(0);
    motor.set_mode(Actuator::SleepMode);
    
    return 0;
}