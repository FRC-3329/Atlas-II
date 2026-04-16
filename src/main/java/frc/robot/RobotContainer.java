package frc.robot;

import frc.robot.constants.OperatorConstants;
import frc.robot.constants.PVConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PDHSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.GameHelpers;

import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
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
    private final SwerveSubsystem drivebase = new SwerveSubsystem();

    // While these cameras aren't referenced directly, they still need to be
    // initalized in the RobotContainer
    @SuppressWarnings("unused")
    private final PhotonVisionSubsystem blueCam = new PhotonVisionSubsystem("Blue_cam",
            PVConstants.A_ROBOT_TO_CAMERA, drivebase::addVisionMeasurement);
    @SuppressWarnings("unused")
    private final PhotonVisionSubsystem redCam = new PhotonVisionSubsystem("Red_cam",
            PVConstants.C_ROBOT_TO_CAMERA, drivebase::addVisionMeasurement);
    @SuppressWarnings("unused")
    private final PhotonVisionSubsystem yellowCam = new PhotonVisionSubsystem("Yellow_cam",
            PVConstants.D_ROBOT_TO_CAMERA, drivebase::addVisionMeasurement);

    private final GameHelpers gameHelpers;

    private final FlywheelSubsystem flywheel;
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final TurretSubsystem turret;

    @SuppressWarnings("unused")
    private final PDHSubsystem pdh = new PDHSubsystem();
    private final LEDSubsystem leds = new LEDSubsystem();

    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    private final Command driveFieldOrientedAngularVelocity;
    private final SwerveInputStream driveAngularVelocity;
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        gameHelpers = new GameHelpers(drivebase::getPose, drivebase::getRobotVelocity);
        flywheel = new FlywheelSubsystem(gameHelpers::getShotParameters);
        turret = new TurretSubsystem(drivebase::getPose, gameHelpers::getVirtualTargetFieldAngle);

        configurePathPlannerCommands();

        drivebase.resetOdometry(new Pose2d(1, 1, Rotation2d.kZero));
        // Coast during setup so the robot can be pushed into position
        setMotorBrake(false);
        driveAngularVelocity = SwerveInputStream
                .of(drivebase.getSwerveDrive(),
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX())
                .withControllerRotationAxis(
                        () -> -driverController.getRightX())
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true)
                .aim(() -> new Pose2d(gameHelpers.getVirtualTargetTranslation(), Rotation2d.kZero))
                .aimHeadingOffset(Rotation2d.fromDegrees(180.0))
                .aimHeadingOffset(true)
                .aimWhile(driverController.b());
        driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

        // Signal readiness to the driver so they know when to shoot
        leds.configureLEDs(() -> turret.isAutoTrackingEnabled()
                && turret.isOnTarget()
                && gameHelpers.isValidShotDistance());

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.setDefaultOption("None", Commands.none());
        autoChooser.addOption("Shoot in Place",
                Commands.parallel(
                        intake.lower().andThen(shoot()),
                        turret.autoTrack())
                        .withTimeout(6.0)
                        .andThen(Commands.sequence(
                                flywheel.stop(), indexer.stop(), intake.stop(),
                                turret.stopAutoTracking())));

        configureBindings();
    }

    /** Provides haptic feedback to alert the driver of state changes. */
    private Command rumbleControllers(double strength, double duration) {
        return Commands
                .run(() -> {
                    driverController.setRumble(RumbleType.kBothRumble, strength);
                })
                .withTimeout(duration)
                .andThen(() -> {
                    driverController.setRumble(RumbleType.kBothRumble, 0);
                })
                .withName("RumbleControllers");
    }

    /** Registers commands that PathPlanner auto routines can reference by name. */
    private void configurePathPlannerCommands() {
        // asProxy() prevents requirement conflicts between the auto command group
        // and the commands that these subsystems schedule elsewhere
        NamedCommands.registerCommand("IntakeGamePiece", intake.intakeForward().asProxy());
        NamedCommands.registerCommand("RaiseIntake", intake.raise().asProxy());
        NamedCommands.registerCommand("KickIntake", intake.kick().asProxy());
        NamedCommands.registerCommand("LowerIntake", intake.lower().asProxy());
        NamedCommands.registerCommand("EjectGamePiece",
                Commands.parallel(
                        intake.intakeBackward().asProxy(),
                        indexer.feedBackwards())
                        .withName("EjectGamePiece"));
        NamedCommands.registerCommand("Shoot", shoot().asProxy());
        NamedCommands.registerCommand("StopFlywheel", flywheel.stop().asProxy());
        NamedCommands.registerCommand("StopIndexer", indexer.stop().asProxy());
        NamedCommands.registerCommand("StopIntake", intake.stop().asProxy());
        NamedCommands.registerCommand("AutoTrackTurret", turret.autoTrack().asProxy());
    }

    /** Button bindings - see CONTROLLER.md for the full layout diagram. */
    private void configureBindings() {
        driverController.leftTrigger(0.5)
                .whileTrue(intake.intakeForward());
        driverController.rightTrigger(0.5)
                .whileTrue(shoot());

        driverController.leftBumper().whileTrue(indexer.feedBackwards().alongWith(intake.intakeBackward()));
        driverController.rightBumper()
                .onTrue(intake.kick())
                .onFalse(intake.lower());

        driverController.a()
                .whileTrue(indexer.feedBackwards());
        // b is bound to swerve aiming above
        driverController.x()
                .whileTrue(drivebase.lockWheels());
        // Rumble confirms the toggle so the driver doesn't have to check the dashboard
        driverController.y()
                .onTrue(flywheel.toggleVaryingRPM()
                        .andThen(rumbleControllers(0.5, 0.25)));

        driverController.povDown()
                .onTrue(intake.lower());
        driverController.povLeft()
                .whileTrue(turret.moveLeft());
        driverController.povRight()
                .whileTrue(turret.moveRight());

        driverController.start()
                .onTrue(turret.autoTrack());
        driverController.back()
                .onTrue(turret.stopAutoTracking());

        // Warn the driver that the goal is about to switch so they can reposition
        // new Trigger(() -> DriverStation.isTeleop() &&
        // gameHelpers.isPhaseShiftImminent())
        // .onTrue(rumbleControllers(1.0, 1.0));
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }

    /**
     * Wraps an auto-driving command to prevent heading-lock snap-back.
     * <p>
     * The drive input stream caches the last heading. Without this wrapper,
     * the robot snaps to the pre-auto heading when the command ends because
     * the heading lock was never updated during auto driving.
     */
    private Command autoDriving(Command drivingCommand) {
        return drivingCommand.beforeStarting(() -> {
            driveAngularVelocity.get();
        }).finallyDo(interrupted -> {
            driveAngularVelocity.get();
        }).withName("AutoDriving");
    }

    /**
     * Spins up the flywheel while simultaneously waiting for target speed,
     * then feeds fuel through the indexer and intake.
     * <p>
     * Falls back to a fixed RPM/angle when varying RPM is disabled
     * (e.g. PhotonVision offline, so distance is unknown).
     */
    public Command shoot() {
        return Commands.parallel(
                Commands.either(flywheel.shoot(), flywheel.shoot(RPM.of(1600.0), Degrees.of(10.0)),
                        flywheel::isVaryingRPMEnabled),
                Commands.waitUntil(flywheel::isAtSpeed).withTimeout(0.4)
                        .andThen(indexer.smartFeed().alongWith(intake.intakeForward())));
    }

    public Command getTurretAutoTrack() {
        return turret.autoTrack();
    }

    public Command getAutonomousCommand() {
        Command auton = autoChooser.getSelected();

        if (auton != null) {
            DogLog.log("Robot/AutoSelected", auton.getName());
            return autoDriving(auton);
        } else {
            DriverStation.reportError(
                    "Auton error: No auton selected :(",
                    false);
            return Commands.none();
        }
    }
}