# 2026 Rebuilt
Source code for the robot used in REBUILT developed by team 3329, the Wildbots!

## Code Outline
Code outline for `src/main/java/frc/robot`

### commands
Custom commands for robot actions

### constants
All constant values
- **Constants.java** - Global constants (hub location, loop time, max speed)
- **FlywheelConstants.java** - PID gains, motion magic parameters, and current limits for flywheel and hood motors
- **IndexerConstants.java** - Motor configuration for note indexing system
- **IntakeConstants.java** - Pivot and roller motor settings for ground intake mechanism
- **OperatorConstants.java** - Controller ports and joystick deadband values
- **PVConstants.java** - PhotonVision AprilTag field layout and vision standard deviations
- **TurretConstants.java** - PID gains, angle limits, and encoder settings for turret rotation

### Main Files
- **Main.java** - Entry point for the robot program
- **Robot.java** - Main robot class with periodic methods and autonomous/teleop initialization
- **RobotContainer.java** - Binds controllers to commands and configures all subsystems

### subsystems
Robot subsystems that control physical mechanisms
- **FlywheelSubsystem.java** - Controls shooter flywheel speed and hood angle with distance-based interpolation maps
- **IndexerSubsystem.java** - Feeds notes from intake mechanism to the shooter
- **IntakeSubsystem.java** - Deploys and retracts the intake arm, runs roller to collect notes from the ground
- **PhotonVisionSubsystem.java** - Processes AprilTag detection data for robot pose estimation
- **SwerveSubsystem.java** - Swerve drive control with field-relative driving, odometry, and pathfinding
- **TurretSubsystem.java** - Rotates turret to aim at targets with auto-tracking capability

### utils
Utility classes for calculations and data structures
- **CircleFitter.java** - Fits circles to sets of points for trajectory calculations
- **GameHelpers.java** - Helper methods for game-specific logic and calculations
- **VisionData.java** - Data structures and interfaces for vision pose estimates

## Contributing
All contributions are welcome! To get started:\
1. Clone the repo
```
# Fork https://github.com/FRC-3329/2026-Rebuilt.git
git clone https://github.com/YOUR_USERNAME/2026-Rebuilt.git
```
2. Add your feature
```
git checkout -b feature-name
git add .
git commit -m "feat: my amazing feature"
```
3. Push
```
git push -u origin feature-name
```
Then make a [pull request!](https://github.com/FRC-3329/2026-Rebuilt/compare)

## Issues
If you find an issue, please check [existing issues](https://github.com/FRC-3329/2026-Rebuilt/issues) first. If your issue is not already filed, feel free to [make one!](https://github.com/FRC-3329/2026-Rebuilt/issues/new)

## Source Code License
This project uses the [WPILib license](./WPILib-License.md)