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

/**
 * This class is where the bulk of the robot should be declared. 
 * Since Command-based is a "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). 
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final SwerveSubsystem drivebase = new SwerveSubsystem();

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(
        OperatorConstants.kDriverControllerPort
    );

    // Drive command
    private final SwerveInputStream driveAngularVelocity;
    private final Command driveFieldOrientedAngularVelocity;

    // Autonomous chooser
    private final SendableChooser<Command> autoChooser;

    // TODO: https://github.com/FRC-3329/2025Reefscape3329/blob/main/src/main/java/frc/robot/RobotContainer.java#L58
    // Looks like we're missing some logic here lol..
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
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

        // Create field-oriented drive command
        driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        // Set default drive command
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

        // Configure motor brake mode (false = coast)
        setMotorBrake(false);

        // Build autonomous chooser from PathPlanner
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        // Default to no autonomous
        autoChooser.setDefaultOption("None", Commands.none());

        // Configure button bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger -> command mappings.
     * Triggers can be created via the {@link edu.wpi.first.wpilibj2.command.button.Trigger#Trigger(java.util.function.BooleanSupplier)}
     * constructor with an arbitrary predicate, or via the named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s 
     * subclasses for {@link CommandXboxController Xbox} controllers.
     */
    private void configureBindings() {
        // Zero gyro on Start button press
        driverController.start().onTrue(drivebase.zeroGyro());

        // Toggle brake mode on Back button press
        driverController.back().onTrue(Commands.runOnce(() -> {
            setMotorBrake(true);
            SmartDashboard.putBoolean("Brake Mode", true);
        }).andThen(Commands.runOnce(() -> {
            setMotorBrake(false);
            SmartDashboard.putBoolean("Brake Mode", false);
        })).repeatedly());
    }

    /**
     * Sets the motor brake mode for the drivetrain.
     * 
     * @param brake true for brake mode, false for coast mode
     */
    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Return the selected autonomous command from the chooser
        // If no auto is selected, this will return Commands.none()
        return autoChooser.getSelected();
    }
}
