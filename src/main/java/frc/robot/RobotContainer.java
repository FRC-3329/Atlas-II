package frc.robot;

import frc.robot.commands.AutoDriveUnderTrenchCommand;
import frc.robot.commands.OrientToHubCommand;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.PVConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    // Subsystems
    private final SwerveSubsystem drivebase = new SwerveSubsystem();
    // private final PhotonVisionSubsystem blueCam = new
    // PhotonVisionSubsystem("blue_cam",
    // PVConstants.BLUE_ROBOT_TO_CAMERA, drivebase::addVisionMeasurement);
    // private final PhotonVisionSubsystem orangeCam = new
    // PhotonVisionSubsystem("orange_cam",
    // PVConstants.ORANGE_ROBOT_TO_CAMERA, drivebase::addVisionMeasurement);
    @SuppressWarnings("unused")
    private final PhotonVisionSubsystem yellowCam = new PhotonVisionSubsystem("Yellow_cam",
            PVConstants.YELLOW_ROBOT_TO_CAMERA, drivebase::addVisionMeasurement);
    @SuppressWarnings("unused")
    private final PhotonVisionSubsystem redCam = new PhotonVisionSubsystem("Red_cam",
            PVConstants.RED_ROBOT_TO_CAMERA, drivebase::addVisionMeasurement);
    private final GameHelpers gameHelpers;
    private final FlywheelSubsystem flywheel;
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final TurretSubsystem turret;
    @SuppressWarnings("unused")
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
        gameHelpers = new GameHelpers(drivebase::getPose, drivebase::getRobotVelocity);
        flywheel = new FlywheelSubsystem(gameHelpers::getShotParameters);
        turret = new TurretSubsystem(drivebase::getPose, gameHelpers::getVirtualTargetFieldAngle);

        configurePathPlannerCommands();
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

        // leds.setDefaultCommand(
        // leds.run(() -> {
        // leds.set(turret.isAutoTrackingEnabled()
        // && turret.isOnTarget()
        // && gameHelpers.isValidShotDistance());
        // }).withName("LEDTurretFeedback"));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.setDefaultOption("None", Commands.none());

        autoChooser.addOption("Shoot in Place",
                Commands.parallel(
                        flywheel.zeroHood().alongWith(intake.lower())
                                .andThen(shoot()),
                        turret.autoTrack())
                        .withTimeout(6.0)
                        .andThen(Commands.sequence(
                                flywheel.stop(), indexer.stop(), intake.stop(), turret.stopAutoTracking())));

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
                intake.intakeForward());
        NamedCommands.registerCommand("RaiseIntake", intake.raise());
        NamedCommands.registerCommand("LowerIntake", intake.lower());
        NamedCommands.registerCommand("EjectGamePiece",
                Commands.parallel(
                        intake.intakeBackward(),
                        indexer.feedBackwards())
                        .withName("EjectGamePiece"));
        NamedCommands.registerCommand("Shoot", shoot());
        NamedCommands.registerCommand("StopFlywheel", flywheel.stop());
        NamedCommands.registerCommand("StopIndexer", indexer.stop());
        NamedCommands.registerCommand("StopIntake", intake.stop());
        NamedCommands.registerCommand("ShootWithAutoTrack",
                shoot().withName("ShootWithAutoTrack"));
        NamedCommands.registerCommand("AutoTrackTurret", turret.autoTrack());
        NamedCommands.registerCommand("ZeroHood", flywheel.zeroHood());

    }

    // See CONTROLLER.md
    private void configureBindings() {
        //// === TRIGGERS === ////
        // Left Trigger: Intake
        driverController.leftTrigger(0.5)
                .whileTrue(intake.intakeForward());
        // Right Trigger: Shoot
        driverController.rightTrigger(0.5)
                .whileTrue(shoot());

        //// === BUMPERS === ////
        // Left Bumper: Align robot to hub
        driverController.leftBumper()
                .whileTrue(new OrientToHubCommand(drivebase, gameHelpers));
        // Right Bumper: Auto drive under trench
        driverController.rightBumper()
                .whileTrue(autoDriving(new AutoDriveUnderTrenchCommand(drivebase, flywheel)));

        //// === FACE BUTTONS === ////
        // A Button: indexer reversal
        driverController.a()
                .whileTrue(indexer.feedBackwards());
        // B Button: intake/indexer reversal
        driverController.b().whileTrue(indexer.feedBackwards().alongWith(intake.intakeBackward()));
        // X Button: Rotate swerve wheels inward (lock wheels)
        driverController.x()
                .whileTrue(autoDriving(new AutoDriveUnderTrenchCommand(drivebase, flywheel)));
        // Y Button: Auto drive to outpost (Not yet implemented)

        //// === D-PAD === ////
        // Up: Move intake up
        driverController.povUp()
                .onTrue(intake.raise());
        // Down: Move intake down
        driverController.povDown()
                .onTrue(intake.lower());
        // Left: Move turret left
        driverController.povLeft()
                .whileTrue(turret.moveLeft());
        // Right: Move turret right
        driverController.povRight()
                .whileTrue(turret.moveRight());

        //// === MENU BUTTONS === ////
        // Start: Start auto turret tracking
        driverController.start()
                .onTrue(turret.autoTrack());
        // Back: Stop auto turret tracking
        driverController.back()
                .onTrue(turret.stopAutoTracking());

        // Vibrate controllers ~4 seconds before a phase shift
        new Trigger(gameHelpers::isPhaseShiftImminent)
                .onTrue(rumbleControllers(1.0, 1.0));
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
     * Uses either distance-based shooting (dynamic) or static shooting based on
     * whether turret auto tracking is enabled. If turret tracking is disabled,
     * positional information cannot be trusted, so static shooting is used.
     * 
     * @return Command to shoot fuel
     */
    public Command shoot() {
        return Commands.parallel(
                Commands.either(flywheel.shoot(), flywheel.tunableShoot(),
                        turret::isAutoTrackingEnabled),
                Commands.waitUntil(flywheel::isAtSpeed).withTimeout(0.4)
                        .andThen(indexer.smartFeed().alongWith(intake.intakeForward())));
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