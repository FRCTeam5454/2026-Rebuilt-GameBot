# Shotmapping Guide: Time of Flight (TOF) and Code Setup

This guide details the procedure for performing **Shotmapping** to collect distance, flywheel speed, hood position, and time of flight (TOF) values, and how to configure them in the robot's code.

---

## 1. How the Shotmapping Command Works

The [`ShootMappingCommand`](file:///c:/Users/steve/FRC/2026-Rebuilt-GameBot/src/main/java/frc/robot/commands/ShootMappingCommand.java) allows manual control of the shooter speeds and hood positions directly from the SmartDashboard/Shuffleboard so you can find the sweet spot for any given distance.

### Code Bindings
In [`RobotContainer.java`](file:///c:/Users/steve/FRC/2026-Rebuilt-GameBot/src/main/java/frc/robot/RobotContainer.java#L311-L313):
- **Operator Controller**: Pressing and holding the **Start** button (`m_xBoxOperator.start()`) runs the `ShootMappingCommand`.

### SmartDashboard NetworkTable Inputs/Outputs
The command reads configuration values from, and writes feedback to, the SmartDashboard:
* **Inputs (Editable on Shuffleboard/SmartDashboard)**:
  * `"Target Speed"`: The target flywheel speed in Revolutions Per Second (RPS) (Default: `40`).
  * `"Hood Target Position"`: The target angle for the hood pivot in degrees (Default: `0`).
  * `"Idle Speed"`: The idle speed in RPS when the shooter is not active (Default: `20`).
* **Outputs**:
  * `"Odom Distance"`: Calculated horizontal distance from the turret center to the Hub target center in meters.

---

## 2. Step-by-Step Shotmapping Procedure

Follow these steps to map the field and gather data points:

### Step A: Setup & Positioning
1. Position the robot at a measured distance from the Hub (e.g., 2.0 meters, 2.5 meters, 3.0 meters, etc.).
2. Turn on the robot and connect to the driver station.
3. Open **Shuffleboard** or **SmartDashboard**. Ensure the inputs `"Target Speed"` and `"Hood Target Position"` are visible on the dashboard. If they are not, you can manually add double values with these exact names.

### Step B: Configure the Shot Test
1. Set the `"Target Speed"` (RPS) and `"Hood Target Position"` (Degrees) on the dashboard to your starting test values.
2. Read the `"Odom Distance"` on the SmartDashboard (or measure the distance physically from the Hub center to the robot's turret pivot center).

### Step C: Capture the Shot and Time of Flight
1. Set up an external high-speed camera (e.g., a phone recording at 120 FPS or 240 FPS) with a clear view of both the shooter exit and the target (Hub).
2. Start the camera recording.
3. Press and hold the **Start button on the Operator Controller** to execute the shot.
   * *What the robot does*: The hood will actuate to `"Hood Target Position"`, the flywheel will spin up to `"Target Speed"`, and once at target speed, it will feed the ball from the hopper to the shooter.
4. Stop the camera recording.

### Step D: Calculate Time of Flight (TOF)
Because Time of Flight is measured externally:
1. Open the recorded video in an editing software or player that allows frame-by-frame scrubbing (like VLC, Avidemux, or similar).
2. Count the number of frames from the exact frame the ball leaves the shooter wheel to the exact frame the ball enters the target or makes contact with the Hub target.
3. Calculate the TOF in seconds:
   $$\text{TOF (seconds)} = \frac{\text{Number of Frames}}{\text{Frame Rate (FPS)}}$$
   * *Example*: If you recorded at 240 FPS and counted 204 frames:
     $$\text{TOF} = \frac{204}{240} = 0.85\text{ seconds}$$

---

## 3. Entering Shotmapping Data into Code

Once you have recorded a set of successful shot parameters, you need to add them to the Lookup Table in [`HubLookUpTable.java`](file:///c:/Users/steve/FRC/2026-Rebuilt-GameBot/src/main/java/frc/robot/subsystems/shooter/HubLookUpTable.java).

1. Open [`HubLookUpTable.java`](file:///c:/Users/steve/FRC/2026-Rebuilt-GameBot/src/main/java/frc/robot/subsystems/shooter/HubLookUpTable.java).
2. Navigate to the `initializeLookupTable()` method.
3. Use the `addEntry` method to add your data points:
   ```java
   // Syntax: addEntry(distanceMeters, shooterSpeedRPS, hoodAngleDegrees, timeOfFlightSeconds);
   addEntry(2.52, 51.0, 0.1, 0.9);
   ```
4. Add entries sorted by distance. The lookup table will automatically perform linear interpolation (`lerp`) for any distance between the points you define.

---

## 4. Verification of Shoot-On-The-Move

The [`ShotOnTheMoveCommand`](file:///c:/Users/steve/FRC/2026-Rebuilt-GameBot/src/main/java/frc/robot/commands/ShotOnTheMoveCommand.java) works by using the Time of Flight (TOF) data points you provide in `HubLookUpTable` to predict where the robot will be when the ball arrives at the target.

### How it operates:
1. **Inputs**: It retrieves the current robot pose and velocity from the drivetrain (`m_swerve`).
2. **Iterative Compensation**:
   * It calculates the field-relative velocity of the turret.
   * It runs a 5-iteration refinement loop to project the virtual turret position based on the ball's Time of Flight (TOF):
     $$\vec{p}_{\text{virtual}} = \vec{p}_{\text{turret}} + \vec{v}_{\text{turret}} \times \text{TOF}$$
   * It fetches the updated TOF for that virtual distance and repeats the loop.
3. **Aim & Speed**:
   * The shooter speed and hood angle are selected based on the virtual distance.
   * The turret is steered to face the target from the future position (so that the robot's velocity vector directs the ball into the hub).
