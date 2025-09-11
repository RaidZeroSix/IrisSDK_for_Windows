# Shock Tester Control System - Simple Console Version

A standalone console application for controlling Iris Dynamics Orca motors to generate controlled shocks through force-based trapezoidal motion profiles.

## Features

- **Standalone Console Application** - No external GUI dependencies
- **Connection Verification** - Comprehensive motor connection testing before operation
- **Force-Based Control** - Uses force commands exclusively with position feedback
- **State Machine Control** - Automated sequence: Home → Accelerate → Brake → Shock → Stabilize
- **Trapezoidal Motion Profile** - Specify acceleration and braking forces in Newtons
- **Simple Keyboard Controls** - [S]tart, [X]Stop, [Q]uit
- **Multiple Repetitions** - Run tests multiple times automatically
- **Real-Time Status Display** - Position, force, velocity, and state updated 10Hz
- **Custom Profiles** - Input your own force parameters
- **Safety Controls** - Emergency stop and connection monitoring

## Setup

1. **Hardware Setup**
   - Connect Orca motor via RS422 serial connection
   - Power on the motor
   - Note the COM port number assigned to your RS422 adapter

2. **Build the Application**
   - Open `ShockTester.sln` in Visual Studio
   - Build in Release mode (x64) for best performance
   - Executable will be in `x64\Release\ShockTester.exe`

3. **Running the Application**
   - Launch ShockTester.exe
   - Enter the motor COM port number when prompted
   - System will verify connection with detailed diagnostics
   - Select test profile and number of repetitions
   - Press 'S' to start test

## How It Works

The system generates shocks through controlled collisions between a moving bed and a mass:

1. **Homing**: System moves to furthest negative position and zeros
2. **Acceleration**: Applies constant positive force to accelerate the bed
3. **Braking**: At critical position, applies negative force to decelerate
4. **Collision**: Bed and mass collide, creating the shock
5. **Stabilization**: PID control returns system to rest position

### State Machine

```
IDLE → HOMING → ACCELERATE → BRAKE → SHOCK_DETECTED → STABILIZE → COMPLETE
                     ↑                                              ↓
                     └──────────────(repeat if needed)──────────────┘
```

## Test Profile Parameters

Each test profile defines:

- **Acceleration Force (N)**: Positive force applied during acceleration phase
- **Braking Force (N)**: Negative force applied during braking phase  
- **Critical Position (mm)**: Position where system switches from acceleration to braking
- **Stabilize Position (mm)**: Target position after shock for stabilization
- **Shock Threshold (N)**: Force spike threshold to detect collision
- **Velocity Threshold (mm/s)**: Velocity reversal threshold for shock detection

## Operating Instructions

1. **Select Test Profile**
   - Use dropdown menu to choose from predefined tests
   - Or click "Load Custom" to use your own profile

2. **Set Repetitions**
   - Use slider to set number of test repetitions (1-100)

3. **Run Test**
   - Press green "Start Test" button
   - Motor will execute the force profile
   - Repetition counter shows progress

4. **Emergency Stop**
   - Press red "EMERGENCY STOP" button at any time
   - Motor will immediately return to sleep mode

## Safety Considerations

- **Always ensure proper mounting** of test equipment before running
- **Set appropriate force limits** in the Settings page
- **Monitor temperature** during extended testing
- **Use emergency stop** if unexpected behavior occurs
- **Start with low-force profiles** when testing new setups

## Included Test Profiles

1. **10g Light Shock**
   - Acceleration: 50N, Braking: -100N
   - Critical Position: 25mm
   - For delicate equipment testing

2. **30g Medium Shock**
   - Acceleration: 150N, Braking: -300N
   - Critical Position: 30mm
   - Standard shock testing

3. **50g Heavy Shock**
   - Acceleration: 250N, Braking: -500N
   - Critical Position: 35mm
   - High-G testing for robust equipment

4. **Custom Profile**
   - Adjustable through GUI sliders
   - Acceleration: 10-500N
   - Braking: -10 to -500N
   - Critical Position: 5-50mm

## Troubleshooting

- **Motor not connecting**: Check COM port number and RS422 cable connection
- **IrisControls not connecting**: Verify virtual COM port setup with com0com
- **No shock detected**: Adjust shock threshold or check for mechanical issues
- **Position overshoot**: Reduce acceleration force or adjust critical position
- **Unstable stabilization**: Tune PID gains in SimplePID class
- **Test timeout**: Increase timeout_ms in profile or check for stuck mechanism

## Technical Details

- **Control Method**: Force control mode exclusively
- **State Machine**: 7-state automated sequence
- **Communication**: MODBUS over RS422 at 1.25 Mbps
- **Update Rate**: 1000 Hz (1ms loop time)
- **Force Resolution**: 1 mN (0.001 N)
- **Position Resolution**: 1 μm (0.001 mm)
- **Velocity Calculation**: Numerical differentiation of position

## Theory of Operation

The shock is generated by the collision between the moving bed and a mass. The magnitude of the shock depends on:

1. **Kinetic Energy at Impact**: Determined by acceleration force and distance
2. **Deceleration Rate**: Controlled by braking force
3. **Mass Ratio**: Between bed and impacting mass
4. **Collision Characteristics**: Elasticity and damping of the collision

The system doesn't directly control the shock magnitude but rather the conditions that lead to it. Fine-tuning the acceleration force, braking force, and critical position allows control over the resulting shock characteristics.