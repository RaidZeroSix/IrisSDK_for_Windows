/**
 * @file ShockTesterGUI.cpp
 * @brief Main GUI application for shock tester using Dear ImGui
 */

#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_opengl3.h"
#include "imgui/implot.h"

// GLFW includes OpenGL headers
#include <GLFW/glfw3.h>

#include "MotionController.h"
#include "ShockProfiles.h"
#include "modbus_client/device_applications/actuator.h"
#include "library_linker.h"

#include <iostream>
#include <vector>
#include <vector>
#include <string>
#include <algorithm>
#include <windows.h>
#include <mmsystem.h>

#pragma comment(lib, "winmm.lib")  // For PlaySound

using namespace std;

// Global state
static Actuator* g_motor = nullptr;
static MotionController* g_controller = nullptr;
static bool g_connected = false;
static vector<ShockProfile> g_profiles;
static int g_selected_profile = 0;
static int g_repetitions = 1;
static bool g_unit_secured_confirmed = false;

// COM port scanning
static vector<int> g_available_ports;
static int g_selected_port = -1;

// Plot data (3 second window at 100Hz display rate)
static const int PLOT_POINTS = 300;
static vector<float> g_time_data;
static vector<float> g_position_data;
static vector<float> g_force_data;
static float g_plot_time = 0;
static int g_plot_index = 0;

// Status colors
static ImVec4 COLOR_GREEN = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
static ImVec4 COLOR_YELLOW = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
static ImVec4 COLOR_RED = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
static ImVec4 COLOR_WHITE = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);

// Window state
static bool g_show_connection_window = true;
static bool g_show_main_window = false;

// Error message
static string g_error_message = "";
static uint32_t g_error_time = 0;

// Function declarations
void ScanCOMPorts();
bool ConnectToMotor(int port);
void DisconnectMotor();
void UpdatePlotData();
void PlayBeep(bool success);
void ShowConnectionWindow();
void ShowMainWindow();
void HandleKeyboard();

// Error handling
void ShowError(const string& message) {
    g_error_message = message;
    g_error_time = GetTickCount();
    PlayBeep(false);
    cout << "ERROR: " << message << endl;
}

// Success notification
void ShowSuccess(const string& message) {
    PlayBeep(true);
    cout << "SUCCESS: " << message << endl;
}

int main(int argc, char** argv) {
    // Setup GLFW
    if (!glfwInit()) {
        cout << "Failed to initialize GLFW" << endl;
        return -1;
    }

    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // Create window
    GLFWwindow* window = glfwCreateWindow(1280, 800, "Shock Tester Control System", NULL, NULL);
    if (!window) {
        cout << "Failed to create GLFW window" << endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Setup Dear ImGui style (dark theme)
    ImGui::StyleColorsDark();
    
    // Customize colors for industrial look
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 5.0f;
    style.FrameRounding = 3.0f;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.1f, 0.1f, 1.0f);
    style.Colors[ImGuiCol_TitleBg] = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
    style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.15f, 0.15f, 0.15f, 1.0f);

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Load profiles
    g_profiles = ShockProfiles::getProfiles();

    // Initialize plot data
    g_time_data.resize(PLOT_POINTS);
    g_position_data.resize(PLOT_POINTS);
    g_force_data.resize(PLOT_POINTS);
    for (int i = 0; i < PLOT_POINTS; i++) {
        g_time_data[i] = i * 0.01f;
        g_position_data[i] = 0;
        g_force_data[i] = 0;
    }

    // Scan for COM ports on startup
    ScanCOMPorts();

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        // Poll events
        glfwPollEvents();
        HandleKeyboard();

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Update motor communication if connected
        if (g_connected && g_motor) {
            // Process MODBUS communication
            g_motor->run_in();
            g_motor->run_out();
            
            // Debug output every second
            static uint32_t last_debug = 0;
            uint32_t now = GetTickCount();
            if (now - last_debug > 1000) {
                float pos = g_motor->get_position_um() / 1000.0f;
                float force = g_motor->get_force_mN() / 1000.0f;
                cout << "Debug - Pos: " << pos << " mm, Force: " << force << " N, Connected: " << g_motor->is_connected() << endl;
                last_debug = now;
            }
            
            // Update controller
            if (g_controller) {
                g_controller->update();
                UpdatePlotData();
                
                // Check for test completion
                static bool was_running = false;
                bool is_running = g_controller->isTestRunning();
                if (was_running && !is_running) {
                    ShowSuccess("Test completed!");
                }
                was_running = is_running;
            }
        }

        // Show appropriate window
        if (g_show_connection_window) {
            ShowConnectionWindow();
        }
        if (g_show_main_window) {
            ShowMainWindow();
        }

        // Show error message if any
        if (!g_error_message.empty()) {
            if (GetTickCount() - g_error_time < 5000) {
                ImGui::SetNextWindowPos(ImVec2(10, 10));
                ImGui::Begin("Error", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize);
                ImGui::TextColored(COLOR_RED, "%s", g_error_message.c_str());
                ImGui::End();
            } else {
                g_error_message.clear();
            }
        }

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    DisconnectMotor();
    
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

void ShowConnectionWindow() {
    ImGui::SetNextWindowPos(ImVec2(50, 50), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(400, 300), ImGuiCond_FirstUseEver);
    
    ImGui::Begin("Motor Connection", &g_show_connection_window);
    
    ImGui::Text("Select COM Port for Motor:");
    ImGui::Separator();
    
    if (ImGui::Button("Refresh Ports")) {
        ScanCOMPorts();
    }
    
    ImGui::BeginChild("PortList", ImVec2(0, 150), true);
    for (size_t i = 0; i < g_available_ports.size(); i++) {
        char label[32];
        sprintf_s(label, "COM%d", g_available_ports[i]);
        if (ImGui::Selectable(label, g_selected_port == (int)i)) {
            g_selected_port = (int)i;
        }
    }
    ImGui::EndChild();
    
    ImGui::Separator();
    
    if (g_selected_port >= 0) {
        if (ImGui::Button("Connect", ImVec2(100, 30))) {
            int port = g_available_ports[g_selected_port];
            if (ConnectToMotor(port)) {
                g_show_connection_window = false;
                g_show_main_window = true;
            }
        }
    }
    
    ImGui::End();
}

void ShowMainWindow() {
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
    
    ImGui::Begin("Shock Tester Control System", &g_show_main_window, 
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
    
    // Connection status
    ImGui::Text("Connection Status:");
    ImGui::SameLine();
    if (g_connected) {
        ImGui::TextColored(COLOR_GREEN, "Connected COM%d", g_motor ? g_available_ports[g_selected_port] : 0);
    } else {
        ImGui::TextColored(COLOR_RED, "Disconnected");
    }
    ImGui::SameLine(ImGui::GetWindowWidth() - 150);
    if (ImGui::Button("Disconnect")) {
        DisconnectMotor();
        g_show_main_window = false;
        g_show_connection_window = true;
    }
    
    ImGui::Separator();
    
    // Test setup section
    ImGui::Text("TEST SETUP");
    
    // Profile selection
    ImGui::Text("Profile:");
    ImGui::SameLine(100);
    ImGui::SetNextItemWidth(200);
    if (ImGui::BeginCombo("##Profile", g_profiles[g_selected_profile].name.c_str())) {
        for (size_t i = 0; i < g_profiles.size(); i++) {
            bool is_selected = (g_selected_profile == (int)i);
            if (ImGui::Selectable(g_profiles[i].name.c_str(), is_selected)) {
                g_selected_profile = (int)i;
            }
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    
    // Show profile details
    ImGui::Text("  Acceleration: %.1f N, Braking: %.1f N", 
                g_profiles[g_selected_profile].acceleration_force_N,
                g_profiles[g_selected_profile].braking_force_N);
    ImGui::Text("  Critical Pos 1: %.1f mm, Critical Pos 2: %.1f mm",
                g_profiles[g_selected_profile].critical_position_1_mm,
                g_profiles[g_selected_profile].critical_position_2_mm);
    
    // Repetitions
    ImGui::Text("Repetitions:");
    ImGui::SameLine(100);
    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("##Reps", &g_repetitions);
    if (g_repetitions < 1) g_repetitions = 1;
    if (g_repetitions > 1000) g_repetitions = 1000;
    ImGui::SameLine();
    ImGui::Text("/ 1000");
    
    // Safety confirmation
    ImGui::Checkbox("Unit is securely fastened to carriage", &g_unit_secured_confirmed);
    
    // Control buttons
    ImGui::Spacing();
    
    bool can_start = g_connected && g_controller && !g_controller->isTestRunning() && g_unit_secured_confirmed;
    
    if (!can_start) ImGui::BeginDisabled();
    if (ImGui::Button("START TEST", ImVec2(150, 40))) {
        if (g_controller->isHomed()) {
            g_controller->startTest(g_profiles[g_selected_profile], g_repetitions);
        } else {
            // Need to home first
            g_controller->startHoming();
            // Will auto-start test after homing
        }
    }
    if (!can_start) ImGui::EndDisabled();
    
    ImGui::SameLine();
    
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.1f, 0.1f, 1.0f));
    if (ImGui::Button("EMERGENCY STOP", ImVec2(150, 40)) || ImGui::IsKeyPressed(ImGuiKey_Space)) {
        if (g_controller) {
            g_controller->emergencyStop();
            ShowError("Emergency Stop Activated!");
        }
    }
    ImGui::PopStyleColor();
    
    ImGui::Separator();
    
    // Status section
    ImGui::Text("STATUS");
    
    if (g_controller) {
        // State display with color coding
        ImGui::Text("State:");
        ImGui::SameLine(100);
        ImVec4 state_color = COLOR_WHITE;
        string state_text = g_controller->getStateString();
        
        if (g_controller->getHomingState() != HOMING_IDLE && g_controller->getHomingState() != HOMING_COMPLETE) {
            state_text = "HOMING: " + g_controller->getHomingStateString();
            state_color = COLOR_YELLOW;
        } else if (g_controller->isTestRunning()) {
            state_color = COLOR_GREEN;
        }
        
        ImGui::TextColored(state_color, "%s", state_text.c_str());
        
        // Repetition counter
        ImGui::Text("Rep:");
        ImGui::SameLine(100);
        ImGui::Text("%d / %d", g_controller->getCurrentRep(), g_controller->getTotalReps());
        
        // Position
        ImGui::Text("Position:");
        ImGui::SameLine(100);
        ImGui::Text("%.2f mm", g_controller->getPosition());
        
        // Force
        ImGui::Text("Force:");
        ImGui::SameLine(100);
        ImGui::Text("%.1f N", g_controller->getForce());
        
        // Velocity
        ImGui::Text("Velocity:");
        ImGui::SameLine(100);
        ImGui::Text("%.1f mm/s", g_controller->getVelocity());
        
        // Travel limits (if homed)
        if (g_controller->isHomed()) {
            ImGui::Text("Travel:");
            ImGui::SameLine(100);
            ImGui::Text("0 to %.1f mm", g_controller->getMaxTravel());
        }
    }
    
    ImGui::Separator();
    
    // Real-time plots
    ImGui::Text("REAL-TIME PLOTS (3 second window)");
    
    if (ImPlot::BeginPlot("Position vs Time", ImVec2(-1, 200))) {
        ImPlot::SetupAxes("Time (s)", "Position (mm)");
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, 3, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -10, 150, ImGuiCond_Always);
        
        if (!g_time_data.empty()) {
            ImPlot::PlotLine("Position", g_time_data.data(), g_position_data.data(), g_time_data.size());
        }
        
        ImPlot::EndPlot();
    }
    
    if (ImPlot::BeginPlot("Force vs Time", ImVec2(-1, 200))) {
        ImPlot::SetupAxes("Time (s)", "Force (N)");
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, 3, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -700, 700, ImGuiCond_Always);
        
        if (!g_time_data.empty()) {
            ImPlot::PlotLine("Force", g_time_data.data(), g_force_data.data(), g_time_data.size());
        }
        
        ImPlot::EndPlot();
    }
    
    ImGui::End();
}

void ScanCOMPorts() {
    g_available_ports.clear();
    
    // Query Windows registry for COM ports
    HKEY hKey;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &hKey) == ERROR_SUCCESS) {
        char valueName[256];
        BYTE valueData[256];
        DWORD valueNameSize, valueDataSize, valueType;
        DWORD index = 0;
        
        while (true) {
            valueNameSize = sizeof(valueName);
            valueDataSize = sizeof(valueData);
            
            if (RegEnumValueA(hKey, index++, valueName, (LPDWORD)&valueNameSize, NULL, &valueType, valueData, (LPDWORD)&valueDataSize) != ERROR_SUCCESS) {
                break;
            }
            
            if (valueType == REG_SZ) {
                string portStr((char*)valueData);
                if (portStr.substr(0, 3) == "COM") {
                    int portNum = atoi(portStr.substr(3).c_str());
                    if (portNum > 0) {
                        g_available_ports.push_back(portNum);
                    }
                }
            }
        }
        RegCloseKey(hKey);
    }
    
    sort(g_available_ports.begin(), g_available_ports.end());
}

bool ConnectToMotor(int port) {
    try {
        // Create motor
        g_motor = new Actuator(port, "Shock Motor", 1);
        
        // Configure connection
        Actuator::ConnectionConfig config;
        config.target_baud_rate_bps = 1250000;
        config.target_delay_us = 0;
        
        g_motor->set_connection_config(config);
        g_motor->set_new_comport(port);
        g_motor->init();
        g_motor->enable();
        
        // Wait for connection
        for (int i = 0; i < 50; i++) {
            Sleep(100);
            g_motor->run_in();
            g_motor->run_out();
            
            if (g_motor->is_connected()) {
                // Success!
                cout << "Motor connected! Setting up..." << endl;
                
                // Enable and configure motor
                g_motor->enable();
                g_motor->set_mode(Actuator::ForceMode);
                g_motor->set_max_force(1000000); // 1000 mN = 1000N max
                
                // Test communication
                float pos = g_motor->get_position_um() / 1000.0f;
                cout << "Initial position: " << pos << " mm" << endl;
                
                // Create controller
                g_controller = new MotionController(*g_motor);
                
                g_connected = true;
                ShowSuccess("Connected to motor!");
                
                // Don't start homing automatically - let user initiate
                cout << "Motor ready. Press START TEST to begin homing." << endl;
                
                return true;
            }
        }
        
        // Failed to connect
        ShowError("Failed to connect to motor on COM" + to_string(port));
        delete g_motor;
        g_motor = nullptr;
        return false;
        
    } catch (...) {
        ShowError("Exception while connecting to motor");
        if (g_motor) {
            delete g_motor;
            g_motor = nullptr;
        }
        return false;
    }
}

void DisconnectMotor() {
    if (g_controller) {
        g_controller->emergencyStop();
        delete g_controller;
        g_controller = nullptr;
    }
    
    if (g_motor) {
        g_motor->set_force_mN(0);
        g_motor->set_mode(Actuator::SleepMode);
        delete g_motor;
        g_motor = nullptr;
    }
    
    g_connected = false;
}

void UpdatePlotData() {
    static uint32_t last_update = 0;
    uint32_t now = GetTickCount();
    
    // Update at 100Hz for display
    if (now - last_update < 10) return;
    last_update = now;
    
    // Circular buffer update
    g_position_data[g_plot_index] = g_controller->getPosition();
    g_force_data[g_plot_index] = g_controller->getForce();
    
    g_plot_index = (g_plot_index + 1) % PLOT_POINTS;
    g_plot_time += 0.01f;
}

void HandleKeyboard() {
    // Space bar for emergency stop is handled in the GUI button code
}

void PlayBeep(bool success) {
    if (success) {
        Beep(800, 200); // Higher pitch for success
    } else {
        Beep(400, 500); // Lower pitch for error
    }
}