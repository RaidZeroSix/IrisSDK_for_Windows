/**
 * @file ShockProfiles.h
 * @brief Hardcoded shock test profiles for the shock tester system
 */

#ifndef SHOCK_PROFILES_H
#define SHOCK_PROFILES_H

#include "MotionController.h"
#include <vector>

// Define all available shock profiles
// These are hardcoded based on testing requirements
class ShockProfiles {
public:
    static std::vector<ShockProfile> getProfiles() {
        std::vector<ShockProfile> profiles;
        
        // Profile 1: Light shock (10g equivalent)
        profiles.push_back({
            "10g Light Shock",
            100.0f,   // acceleration_force_N
            -200.0f,  // braking_force_N
            60.0f,    // critical_position_1_mm (start braking)
            40.0f,    // critical_position_2_mm (end braking)
            50.0f,    // stabilization_position_mm
            1000.0f   // stabilization_time_ms
        });
        
        // Profile 2: Medium shock (30g equivalent)
        profiles.push_back({
            "30g Medium Shock",
            200.0f,   // acceleration_force_N
            -400.0f,  // braking_force_N
            70.0f,    // critical_position_1_mm
            35.0f,    // critical_position_2_mm
            50.0f,    // stabilization_position_mm
            1000.0f   // stabilization_time_ms
        });
        
        // Profile 3: Heavy shock (50g equivalent)
        profiles.push_back({
            "50g Heavy Shock",
            300.0f,   // acceleration_force_N
            -600.0f,  // braking_force_N (motor will saturate at max)
            80.0f,    // critical_position_1_mm
            30.0f,    // critical_position_2_mm
            50.0f,    // stabilization_position_mm
            1000.0f   // stabilization_time_ms
        });
        
        // Profile 4: Custom test profile
        profiles.push_back({
            "Custom Test Profile",
            150.0f,   // acceleration_force_N
            -300.0f,  // braking_force_N
            65.0f,    // critical_position_1_mm
            38.0f,    // critical_position_2_mm
            45.0f,    // stabilization_position_mm
            1500.0f   // stabilization_time_ms
        });
        
        // Profile 5: Low force test
        profiles.push_back({
            "Low Force Test",
            50.0f,    // acceleration_force_N
            -100.0f,  // braking_force_N
            50.0f,    // critical_position_1_mm
            45.0f,    // critical_position_2_mm
            48.0f,    // stabilization_position_mm
            800.0f    // stabilization_time_ms
        });
        
        // Profile 6: High acceleration test
        profiles.push_back({
            "High Acceleration Test",
            400.0f,   // acceleration_force_N (will use motor max)
            -400.0f,  // braking_force_N
            75.0f,    // critical_position_1_mm
            25.0f,    // critical_position_2_mm
            50.0f,    // stabilization_position_mm
            1200.0f   // stabilization_time_ms
        });
        
        // Add more profiles as needed
        // These values should be tuned based on actual testing
        // to achieve desired shock characteristics
        
        return profiles;
    }
    
    static ShockProfile getProfileByIndex(size_t index) {
        auto profiles = getProfiles();
        if (index < profiles.size()) {
            return profiles[index];
        }
        // Return default profile if index out of range
        return profiles[0];
    }
    
    static ShockProfile getProfileByName(const std::string& name) {
        auto profiles = getProfiles();
        for (const auto& profile : profiles) {
            if (profile.name == name) {
                return profile;
            }
        }
        // Return default profile if not found
        return profiles[0];
    }
    
    static size_t getProfileCount() {
        return getProfiles().size();
    }
};

#endif // SHOCK_PROFILES_H