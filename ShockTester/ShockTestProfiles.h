/**
    @file ShockTestProfiles.h
    @brief Test profile definitions for shock tester
    @version 2.0
*/

#ifndef SHOCK_TEST_PROFILES_H
#define SHOCK_TEST_PROFILES_H

#include <string>

/**
 * @struct ShockTestProfile
 * @brief Defines parameters for a shock test using force-based control
 */
struct ShockTestProfile {
    // Profile identification
    std::string name;
    std::string description;
    
    // Force parameters (in Newtons)
    float acceleration_force_N;  // Positive force for acceleration phase
    float braking_force_N;       // Negative force for braking phase
    
    // Position parameters (in mm)
    float critical_position_mm;  // Position to switch from acceleration to braking
    float stabilize_position_mm; // Final position setpoint after shock
    
    // Detection parameters
    float shock_threshold_N;     // Force spike threshold for shock detection
    float velocity_threshold_mms; // Velocity threshold for reversal detection
    
    // Safety parameters
    float max_position_mm;       // Absolute position limit for safety
    float timeout_ms;           // Maximum time before test abort
};

#endif // SHOCK_TEST_PROFILES_H