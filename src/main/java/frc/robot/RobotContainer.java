package frc.robot;

import frc.robot.commands.OrientToHubCommand;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.GameHelpers;

import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    // Subsystems
    private final SwerveSubsystem drivebase = new SwerveSubsystem();
    private final GameHelpers gameHelpers;
    private final FlywheelSubsystem flywheel;
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final TurretSubsystem turret;
    private final LEDsSubsystem leds = new LEDsSubsystem();

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);

    // Commands
    private final Command driveFieldOrientedAngularVelocity;

    private final SwerveInputStream driveAngularVelocity;
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        gameHelpers = new GameHelpers(() -> drivebase.getSwerveDrive().getPose());
        flywheel = new FlywheelSubsystem(gameHelpers::getHubDistance);
        turret = new TurretSubsystem(drivebase, gameHelpers);

        drivebase.resetOdometry(new Pose2d(1, 1, Rotation2d.kZero));

        // Configure motor brake mode (false = coast)
        setMotorBrake(false);

        // Configure drive input stream with deadband and alliance-relative control
        driveAngularVelocity = SwerveInputStream
                .of(drivebase.getSwerveDrive(),
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX())
                .withControllerRotationAxis(
                        () -> -driverController.getRightX())
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);
        driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

        leds.setDefaultCommand(
                leds.run(() -> {
                    leds.set(turret.isAutoTrackingEnabled() && turret.isOnTarget());
                }).withName("LEDTurretFeedback"));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.setDefaultOption("None", Commands.none());

        configurePathPlannerCommands();
        configureBindings();
    }

    /**
     * @param strength rumble strength from 0 to 1
     * @param duration how many seconds to rumble the controllers for
     * @return the command to rumble the controllers
     */
    private Command rumbleControllers(double strength, double duration) {
        return Commands
                .run(() -> {
                    driverController.setRumble(RumbleType.kBothRumble, strength);
                    operatorController.setRumble(RumbleType.kBothRumble, strength);
                })
                .withTimeout(duration)
                .andThen(() -> {
                    driverController.setRumble(RumbleType.kBothRumble, 0);
                    operatorController.setRumble(RumbleType.kBothRumble, 0);
                })
                .withName("RumbleControllers");
    }

    private void configurePathPlannerCommands() {
        NamedCommands.registerCommand("IntakeGamePiece", 
            Commands.parallel(
                intake.lower(),
                intake.intakeForward())
                .withName("IntakeGamePiece"));    
        NamedCommands.registerCommand("RaiseIntake", intake.raise());  
        NamedCommands.registerCommand("EjectGamePiece",
            Commands.parallel(
                intake.intakeBackward(),
                indexer.feed())
                .withName("EjectGamePiece"));   
        NamedCommands.registerCommand("Shoot", shoot());
        NamedCommands.registerCommand("ShootWithAutoTrack",
            Commands.parallel(
                shoot(),
                turret.autoTrack())
                .withName("ShootWithAutoTrack"));
        NamedCommands.registerCommand("AutoTrackTurret", turret.autoTrack());
    }

    // See CONTROLLER.md
    private void configureBindings() {
        driverController.start().onTrue(drivebase.zeroGyro());
 
        driverController.back().onTrue(Commands.runOnce(() -> {
            setMotorBrake(true);
            SmartDashboard.putBoolean("Brake Mode", true);
        }).andThen(Commands.runOnce(() -> {
            setMotorBrake(false);
            SmartDashboard.putBoolean("Brake Mode", false);
        })).repeatedly().withName("ToggleBrakeMode"));

        driverController.leftBumper()
                .whileTrue(Commands.parallel(
                        intake.lower(),
                        intake.intakeForward())
                        .withName("IntakeGamePiece"));

        driverController.rightBumper()
                .whileTrue(Commands.parallel(
                        shoot(),
                        turret.autoTrack())
                        .withName("ShootWithAutoTrack"));

        driverController.leftTrigger(0.5)
                .whileTrue(new OrientToHubCommand(drivebase, gameHelpers));

        driverController.rightTrigger(0.5)
                .whileTrue(turret.autoTrack());

        driverController.a()
                .onTrue(intake.raise());

        driverController.b()
                .whileTrue(Commands.parallel(
                        intake.intakeBackward(),
                        indexer.feed())
                        .withName("EjectGamePiece"));

        driverController.x()
                .whileTrue(turret.moveLeft());

        driverController.y()
                .whileTrue(turret.moveRight());
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }

    /**
     * This is required because it prevents the drivetrain from snapping back
     * to the angle it was at before starting the auto driving command.
     * This is required due to how {@code translationOnlyWhile() } works and
     * how the controller processor updates the heading lock.
     * 
     * @param drivingCommand The command to auto drive.
     * @return The wrapped command to auto drive.
     */
    private Command autoDriving(Command drivingCommand) {
        return drivingCommand.beforeStarting(() -> {
            // Update state in controller supplier
            driveAngularVelocity.get();
        }).finallyDo(interrupted -> {
            driveAngularVelocity.get();
        }).withName("AutoDriving");
    }

    /**
     * 1. Spin up the flywheel
     * 2. In parallel, wait until the flywheel is at speed, then run the indexer and
     * intake
     * 
     * @return Command to shoot fuel
     */
    public Command shoot() {
        return Commands.parallel(
                // Continuously run the flywheel
                flywheel.shoot(),
                // Wait for flywheel to reach speed, then feed
                Commands.sequence(
                        Commands.waitUntil(flywheel::isAtSpeed),
                        Commands.parallel(
                                indexer.feed(),
                                intake.intakeForward())))
                .withName("Shoot");
    }

    /**
     * @return Command to auto-track the turret to the target
     */
    public Command getTurretAutoTrack() {
        return turret.autoTrack();
    }

    public Command getAutonomousCommand() {
        Command auton = autoChooser.getSelected();

        if (auton != null) {
            return autoDriving(auton);
        } else {
            DriverStation.reportError(
                    "Auton error: No auton selected :(",
                    false);
            return Commands.none();
        }
    }
}