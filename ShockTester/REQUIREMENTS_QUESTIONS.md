# Shock Tester Requirements & Design Questions

Please answer each question to help define the system behavior and requirements.

## 1. Physical System Configuration

### 1.1 Mechanical Setup
- **Q1.1.1:** Describe the physical arrangement. Is the motor moving a carriage/bed that impacts a separate mass, or is it accelerating a mass that impacts a fixed stop?
- **Answer:** The motor is moving a carriage that has a mass in it which can move freely. When that mass is at the end of it's stroke it can be pulled along by the carriage. Once the carriage deccelerates, the mass continues along it's stroke and will collide with a rigid bumper on the carriage. This generates a shock like force (short impulsive acceleration)

- **Q1.1.2:** What is the total travel distance available (in mm)?
- **Answer:** The total travel that the motor can move the carriage is 139mm. Homing is necessary and truthfully it would make sense if we do a "mapping" run at the start where we move the carriage/motor at a fixed speed and run into the end stops and see what the encoder says. That way we don't rely on a hard coded value.

- **Q1.1.3:** What are the typical masses involved (bed mass, test specimen mass range)?
- **Answer:** Not relevant/necessary for the development of this program.

- **Q1.1.4:** Are there physical end stops at both extremes of travel?
- **Answer:** Yes, there are physical end stops for the carriage/motor. They are about 139mm apart but we should measure that with the encoders by hitting the far endstop, zero at that location, then hit the other endstop and note it's location in mm.

- **Q1.1.5:** Is there any compliance/spring/damper in the system, or is it a rigid impact?
- **Answer:** Not relevant but no.

### 1.2 Coordinate System
- **Q1.2.1:** When facing the system, which direction should be considered positive? (toward motor, away from motor, left, right, up, down?)
- **Answer:** The positive direction should be toward the motor. Therefore, the zero location should be at the furthest position from the motor, then the motor pulls the carriage towards itself in the positive direction.

- **Q1.2.2:** Where should "zero" position ideally be? (center of travel, one extreme, impact point?)
- **Answer:** At the furthest extreme.

- **Q1.2.3:** Should positions be displayed in mm from zero, or as absolute encoder counts?
- **Answer:** Should be displayed in mm from zero.

## 2. Homing Procedure

### 2.1 Homing Sequence
- **Q2.1.1:** Should homing always go to the same end (e.g., always negative direction first)?
- **Answer:** Yes homing should always go to the same end. As for which, it should be the end stop that we've determined as the zero position.

- **Q2.1.2:** What force threshold indicates we've hit an end stop? (e.g., 50N, 100N?)
- **Answer:** Perhaps we should home at a constant velocity (30mm/s) and once we notice that the velocity won't change we can say "we've arrived to an end stop"

- **Q2.1.3:** How long should we wait at a force threshold before declaring "home found"? (e.g., 100ms, 500ms?)
- **Answer:** 750ms sounds about fine.

- **Q2.1.4:** After finding home, where should the system move to be "ready"? (stay at home, move to center, move to start position?)
- **Answer:** The system should always begin all of it's executed shock profiles from zero (the furthest end stop)

### 2.2 Homing Behavior
- **Q2.2.1:** What should happen if homing fails? (retry automatically, require operator intervention, emergency stop?)
- **Answer:** If homing fails, we should notify the operator and allow them to inspect and hit retry.

- **Q2.2.2:** Should homing speed be configurable or fixed? If fixed, what speed?
- **Answer:** Fixed at 30 mm/s

- **Q2.2.3:** Should the system remember its position between power cycles, or always require homing on startup?
- **Answer:** Always should require homing on startup.

## 3. Shock Test Motion Profile

### 3.1 Motion Sequence
- **Q3.1.1:** Describe the ideal motion sequence step by step (e.g., "move to -30mm, accelerate toward +30mm, brake at 0mm, collide at +10mm")
- **Answer:** The ideal motion sequence would begin at 0mm (the furthest end stop), then the motor will pull the carriage with the force specified by the profile until they reach a critical position, say 60mm. At the critical position we should apply a braking force with the magnitude specified in the profile. Now, that braking force will continue to be applied until the carriage crosses a second critical position. Once it does hit that position, we should stablize to another specific position (supplied by the profile) and we'll need PID for this of course.

So we would Start from zero -> Accelerate until we cross critical position -> apply a reverse braking force until we cross a second critical position (from the correct direction) -> Stablize the system -> go back home -> repeat.

- **Q3.1.2:** Should the acceleration phase use constant force or ramped force?
- **Answer:** Constant force

- **Q3.1.3:** Should braking happen at a fixed position or based on velocity?
- **Answer:** Braking (negative force, not just slowing down) should begin at a fixed position and should proceed until we cross a second critical position

- **Q3.1.4:** After impact, should the system actively return to start position or just stop?
- **Answer:** We won't know when impact happens. Can't be measured really.

### 3.2 Force Calculations
- **Q3.2.1:** How should forces be specified? (direct force in Newtons, G-levels with mass input, percentage of max force?)
- **Answer:** Profiles aren't going to be made by operators. They are going to be made by me so no need to make an interface for this. That said, I'll be using force in Newtons.

- **Q3.2.2:** Should operators input specimen mass for automatic force calculation?
- **Answer:** No. I'll handle that in the profile. They will just select the profile they want to simulate.

- **Q3.2.3:** What's the maximum safe force for your system?
- **Answer:** The motor can output 426 Newtons to a speed of 2 m/s and then it linearly drops off in force that it can supply until about 4 m/s where it supplies 0 Newtons of force.

However this doesn't matter because the kinematics work out such that the motor/carriage won't ever move faster than 2 m/s so the peak force is effectively 426 N. However that is what we command so if we command 500 N that should be fine too, the motor will just send max force. At least we believe

- **Q3.2.4:** Should there be different force limits for different directions?
- **Answer:** No. We won't need to enforce this anyway.

### 3.3 Test Parameters
- **Q3.3.1:** What parameters should operators be able to adjust for each test?
- **Answer:** Operators should really only be able to select a profile and the number of repetitions they want to run. (the number of times they want the product shocked by the selected profile)

- **Q3.3.2:** Should there be preset test profiles? If so, what standards? (MIL-STD-810, IEC, custom?)
- **Answer:** Well, yeah. I'm going to be making them.

- **Q3.3.3:** How many repetitions are typically needed? (1-10, 10-100, 100+?)
- **Answer:** As many reps as desired by the operator. However let's make a limit of 1000 repetitions just for the sake of adding a ceiling.

- **Q3.3.4:** Should there be a delay between repetitions? If so, how long?
- **Answer:** There really shouldn't be a delay. The delay will come from the homing procedure. I.e. when homing is complete after a rep, then we would do another rep.

## 4. Operator Interface & Workflow

### 4.1 Test Setup Workflow
- **Q4.1.1:** What's the ideal step-by-step procedure for operators? List each step.
- **Answer:** Start the application, establish a connection to the motor, select a profile, set the number of reps and then start with a read out of the number of repetitions completed thus far. As well as an emergency stop. Perhaps press spacebar as an estop?

- **Q4.1.2:** What information must operators enter before running a test?
- **Answer:** Just the profile and the number of reps.

- **Q4.1.3:** Should operators need to confirm safety checks before starting?
- **Answer:** Actually, yes. They should be asked "Is the unit securely fastened to the carriage?" before they can press start.

- **Q4.1.4:** Should there be different operator permission levels? (operator vs engineer vs admin?)
- **Answer:** Nah. One level is fine.

### 4.2 Display Requirements
- **Q4.2.1:** What information is critical to display during a test?
- **Answer:** The number of reps and the profile we are testing. Also, the current force of the motor, the position of the motor are useful to see.

- **Q4.2.2:** What parameters should be shown in real-time vs. post-test?
- **Answer:** I mean there really isn't a difference? Maybe just a message stating that the run has been completed?

- **Q4.2.3:** Should the GUI show position in mm, percentage of travel, or both?
- **Answer:** mm. Not percentage of travel.

- **Q4.2.4:** What status messages/warnings are important?
- **Answer:** Errors of all kinds are important.

### 4.3 Controls
- **Q4.3.1:** Besides Start/Stop/E-Stop, what controls do operators need?
- **Answer:** Nothing else I suppose.

- **Q4.3.2:** Should there be a manual jog mode for positioning?
- **Answer:** Yeah, we could add one I guess. Not a high priority though.

- **Q4.3.3:** Should operators be able to abort mid-test and save partial data?
- **Answer:** No need to save partial data.

- **Q4.3.4:** How should the system indicate it's safe to load/unload specimens?

## 5. Data & Results

### 5.1 Data Recording
- **Q5.1.1:** What data should be recorded during each test? (time, position, force, velocity, calculated acceleration?)
- **Answer:** None of this data *needs* to be recorded per se. As in, for later usage. If anything, we can record position and force. From that I can find everything else if needed.

- **Q5.1.2:** What sampling rate is needed? (100 Hz, 1000 Hz, as fast as possible?)
- **Answer:** as fast as possible would be nice.

- **Q5.1.3:** Should data be saved automatically or require operator action?
- **Answer:** operator action.

- **Q5.1.4:** What file format is preferred? (CSV, Excel, TDMS, custom?)
- **Answer:** CSV is fine.

### 5.2 Results Display
- **Q5.2.1:** What results should be calculated and displayed after each test?
- **Answer:** Nothing really.

- **Q5.2.2:** Should the system calculate peak G-force even without an accelerometer?
- **Answer:** No. We've got an accelerometer but it is hooked up to a different app. Also, the system will follow a profile specified in the program which will reliably yield a certain shock. So we don't need to worry about that. Think open loop on that.

- **Q5.2.3:** Should results be compared to pass/fail criteria?
- **Answer:** No. Or at least not applicable right now.

- **Q5.2.4:** How should multiple repetition results be summarized?
- **Answer:** Not applicable at this time.

## 6. Safety & Error Handling

### 6.1 Safety Limits
- **Q6.1.1:** What are the absolute position limits that should never be exceeded?
- **Answer:** The end stops that we find at the start. The far and close one.

- **Q6.1.2:** What's the maximum velocity the system should allow?
- **Answer:** No maximum.

- **Q6.1.3:** Should there be different limits for different test types?
- **Answer:** The motor should just follow the profile.

- **Q6.1.4:** What should trigger an automatic emergency stop?
- **Answer:** The user pressing the e stop or a failure to home.

### 6.2 Error Conditions
- **Q6.2.1:** How should the system respond to communication loss with the motor?
- **Answer:** Report a loss of communication and end whatever test. Not  much more can be done anyway.

- **Q6.2.2:** What should happen if position limits are exceeded?
- **Answer:** They shouldn't be exceeded given the profile I make.

- **Q6.2.3:** How should the system handle unexpected position changes (e.g., manual movement)?
- **Answer:** There won't be any.

- **Q6.2.4:** Should errors be logged to a file for debugging?
- **Answer:** No, just report them in the UI.

## 7. GUI Specific Requirements

### 7.1 Layout Preferences
- **Q7.1.1:** Single window or multiple windows/tabs?
- **Answer:** Single window.

- **Q7.1.2:** Preferred window size/resolution?
- **Answer:** Undetermined

- **Q7.1.3:** Dark theme, light theme, or user selectable?
- **Answer:** Dark theme.

- **Q7.1.4:** Should GUI scale for different monitor sizes?
- **Answer:** I suppose.

### 7.2 Real-time Plots
- **Q7.2.1:** What should be plotted in real-time? (position vs time, force vs time, force vs position?)
- **Answer:** Already answered, Position vs time and force vs time.

- **Q7.2.2:** How much history should plots show? (last 5 seconds, entire test, scrolling window?)
- **Answer:** last 3 seconds.

- **Q7.2.3:** Should plots be able to be saved as images?
- **Answer:** Not necessary.

- **Q7.2.4:** Should operators be able to zoom/pan plots?
- **Answer:** Also not necessary.

### 7.3 User Feedback
- **Q7.3.1:** How should the system indicate test progress? (progress bar, step indicator, time remaining?)
- **Answer:** Display what rep we are on and what step in the process of that rep, we are on.

- **Q7.3.2:** Should there be audio feedback? (beep on test complete, alarm on error?)
- **Answer:** Yes, beep on complete or error.

- **Q7.3.3:** What color coding makes sense? (green=ready, yellow=testing, red=error?)
- **Answer:** Yeah, looks good.

- **Q7.3.4:** Should there be confirmation dialogs, or trust operators to not make mistakes?
- **Answer:** Trust operators to not make mistakes.

## 8. Future Considerations

### 8.1 Expansion
- **Q8.1.1:** Might you need to control multiple motors/stations in the future?
- **Answer:** Perhaps but that is for the future. Not a concern at the moment.

- **Q8.1.2:** Will you need to integrate with other test equipment? (DAQ, accelerometers, cameras?)
- **Answer:** We do use an endaq but that will be a later version.

- **Q8.1.3:** Should test profiles be shareable between multiple systems?
- **Answer:** Not necessary. Profiles are going to be hardcoded.

- **Q8.1.4:** Will you need remote monitoring capability?
- **Answer:** No.

### 8.2 Validation
- **Q8.2.1:** How will you validate that shocks are correct? (external accelerometer, high-speed camera?)
- **Answer:** I have an accelerometer that I will use to check what the peak shock is at any time.

- **Q8.2.2:** Do you need traceability/audit trails for test records?
- **Answer:** Not really.

- **Q8.2.3:** Should the system generate test reports automatically?
- **Answer:** No.

- **Q8.2.4:** Are there regulatory requirements to consider?
- **Answer:** No.

## 9. Specific Behavioral Questions

### 9.1 Edge Cases
- **Q9.1.1:** If a test is started but homing hasn't been done, should it auto-home first or refuse to start?
- **Answer:** Homing is part of a test. A test can't happen without the homing step. It's sort of like a state machine.

- **Q9.1.2:** If emergency stop is pressed mid-test, should partial data be saved?
- **Answer:** Not needed.

- **Q9.1.3:** If the same test is run multiple times, should data append or overwrite?
- **Answer:** There isn't really any data to capture.

- **Q9.1.4:** If connection is lost during a test, should the system try to reconnect automatically?
- **Answer:** No, just go back to the menu/state where a connection hasn't been made yet.

### 9.2 User Experience
- **Q9.2.1:** Should settings persist between sessions? (last used profile, window positions, etc.)
- **Answer:** No.

- **Q9.2.2:** Should there be keyboard shortcuts for common operations?
- **Answer:** No.

- **Q9.2.3:** Should the system prevent simultaneous operations (e.g., can't change settings while testing)?
- **Answer:** Yes. Once a test is under way, nothing can be changed without an stopping the test.

- **Q9.2.4:** How verbose should status messages be? (minimal, normal, detailed/debug?)
- **Answer:** Decently verbose.

### 9.3 Performance
- **Q9.3.1:** Is the current 1ms control loop (1000 Hz) sufficient?
- **Answer:** Yes, this should be sufficient.

- **Q9.3.2:** How quickly must the GUI respond to feel responsive? (instant, <100ms, <1second?)
- **Answer:** As quick as possible.

- **Q9.3.3:** How important is deterministic timing for the motion profile?
- **Answer:** Very important.

- **Q9.3.4:** Should the system prioritize smooth motion or exact timing?
- **Answer:** Exact timing.
---

Please provide answers to help define the system requirements. Feel free to skip questions that aren't relevant or add additional context where needed.