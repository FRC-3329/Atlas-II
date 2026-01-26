package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

import swervelib.SwerveInputStream;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
    TODO:
        - Integrate QN subsystem
        - Operator controller
            - Configure controller bindings
            - Rumble
            - SwerveInputStream :(
        - Auto driving (?)
            - https://github.com/FRC-3329/2025Reefscape3329/blob/main/src/main/java/frc/robot/RobotContainer.java#L261
        - Integrate PV subsystem (once coded)
            - QN -> PV Failover system
        - Integrate all game-specific subsystems and commands (once coded)
        - resetOdometry()
        - Reset odometry to PV if not connected to FMS (?)
            - https://github.com/FRC-3329/2025Reefscape3329/blob/main/src/main/java/frc/robot/RobotContainer.java#L61
        - SmartDashboard integration
 */

public class RobotContainer {
    // Subsystems
    private final SwerveSubsystem drivebase = new SwerveSubsystem();

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(
        OperatorConstants.kDriverControllerPort
    );

    // Commands
    private final SwerveInputStream driveAngularVelocity;
    private final Command driveFieldOrientedAngularVelocity;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Configure drive input stream with deadband and alliance-relative control
        driveAngularVelocity = SwerveInputStream
                .of(drivebase.getSwerveDrive(), 
                    () -> -driverController.getLeftY(), 
                    () -> -driverController.getLeftX()
                )
                .withControllerRotationAxis(() -> -driverController.getRightX())
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);

        driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

        // Configure motor brake mode (false = coast)
        setMotorBrake(false);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.setDefaultOption("None", Commands.none());

        configureBindings();
    }

    private void configureBindings() {
        driverController.start().onTrue(drivebase.zeroGyro());

        driverController.back().onTrue(Commands.runOnce(() -> {
            setMotorBrake(true);
            SmartDashboard.putBoolean("Brake Mode", true);
        }).andThen(Commands.runOnce(() -> {
            setMotorBrake(false);
            SmartDashboard.putBoolean("Brake Mode", false);
        })).repeatedly());
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
