# 2026 Rebuilt
Source code for the robot used in REBUILT developed by team 3329, the Wildbots!

## Prerequisites
- **WPILib 2026** - [Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
    - Includes all needed dependencies

## Building and Deploying
### Build the Project
```sh
./gradlew build
```

### Deploy to Robot
```sh
./gradlew deploy
```

## Code Outline
Code outline for `src/main/java/frc/robot`

### `commands`
Custom commands for robot actions
- **AutoDriveUnderTrenchCommand.java** - Automatically pathfinds the robot to and drives under the nearest trench using PathPlanner
- **OrientToHubCommand.java** - Automatically orients the robot so the rear-mounted turret faces the hub for shooting

### `constants`
All constant values
- **Constants.java** - Global constants (hub location, loop time, max speed)
- **FlywheelConstants.java** - PID gains, motion magic parameters, and current limits for flywheel and hood motors
- **IndexerConstants.java** - Motor configuration for note indexing system
- **IntakeConstants.java** - Pivot and roller motor settings for ground intake mechanism
- **LEDConstants.java** - PWM port and strip length for addressable LEDs
- **OperatorConstants.java** - Controller ports and joystick deadband values
- **OrientToHubConstants.java** - PID gains, velocity/acceleration limits, and tolerance for the orient-to-hub command
- **PVConstants.java** - PhotonVision AprilTag field layout and vision standard deviations
- **TrenchConstants.java** - Field dimensions, path names, and motion constraints for trench driving
- **TurretConstants.java** - PID gains, angle limits, and encoder settings for turret rotation

### Main Files
- **Main.java** - Entry point for the robot program
- **Robot.java** - Main robot class with periodic methods and autonomous/teleop initialization
- **RobotContainer.java** - Binds controllers to commands and configures all subsystems

### `subsystems`
Robot subsystems that control physical mechanisms
- **FlywheelSubsystem.java** - Controls shooter flywheel speed and hood angle with distance-based interpolation maps
- **IndexerSubsystem.java** - Feeds notes from intake mechanism to the shooter
- **IntakeSubsystem.java** - Deploys and retracts the intake arm, runs roller to collect notes from the ground
- **LEDSubsystem.java** - Controls addressable LED strip with pattern-based status indicators
- **PDHSubsystem.java** - Manages the Power Distribution Hub switchable channel and logging
- **PhotonVisionSubsystem.java** - Processes AprilTag detection data for robot pose estimation
- **SwerveSubsystem.java** - Swerve drive control with field-relative driving, odometry, and pathfinding
- **TurretSubsystem.java** - Rotates turret to aim at targets with auto-tracking capability

### `utils`
Utility classes for calculations and data structures
- **GameHelpers.java** - Helper methods for game-specific logic and calculations
- **ShootOnTheMove.java** - Computes shot parameters for a moving robot using time-of-flight recursion
- **ShotParameters.java** - Distance-keyed interpolation map of flywheel speed, hood angle, and time-of-flight values
- **VisionData.java** - Data structures and interfaces for vision pose estimates

## Dependencies
- **YAGSL** - Yet Another Generic Swerve Library
- **PathplannerLib** - Path planning and autonomous
- **PhotonLib** - Vision processing with PhotonVision
- **REVLib** - REV Robotics motor controllers
- **Phoenix5/Phoenix6** - CTRE motor controllers
- **DogLog** - Advanced logging framework

## Contributing
All contributions are welcome! To get started:
1. Clone the repo
```sh
# Fork https://github.com/FRC-3329/2026-Rebuilt.git
git clone https://github.com/YOUR_USERNAME/2026-Rebuilt.git
```
2. Add your feature
```sh
git checkout -b feature-name
git add .
git commit -m "feat: my amazing feature"
```
3. Push
```sh
git push -u origin feature-name
```
Then make a [pull request!](https://github.com/FRC-3329/2026-Rebuilt/compare)

## Issues
If you find an issue, please check [existing issues](https://github.com/FRC-3329/2026-Rebuilt/issues) first. If your issue is not already filed, feel free to [make one!](https://github.com/FRC-3329/2026-Rebuilt/issues/new)

## Source Code License
This project uses the [WPILib license](./WPILib-License.md)
