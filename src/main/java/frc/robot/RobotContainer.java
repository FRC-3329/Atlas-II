package frc.robot;

import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.PVConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PDHSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.utils.GameHelpers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
    private final DriveSubsystem drive;

    // While these cameras aren't referenced directly, they still need to be
    // initalized in the RobotContainer
    @SuppressWarnings("unused")
    private final PhotonVisionSubsystem blueCam;
    @SuppressWarnings("unused")
    private final PhotonVisionSubsystem redCam;
    @SuppressWarnings("unused")
    private final PhotonVisionSubsystem yellowCam;

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
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new DriveSubsystem(
                        new GyroIOPigeon2(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3));
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new DriveSubsystem(
                        new GyroIO() {
                        },
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new DriveSubsystem(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
                break;
        }

        blueCam = new PhotonVisionSubsystem("Blue_cam",
                PVConstants.A_ROBOT_TO_CAMERA, drive::addVisionMeasurement);
        redCam = new PhotonVisionSubsystem("Red_cam",
                PVConstants.C_ROBOT_TO_CAMERA, drive::addVisionMeasurement);
        yellowCam = new PhotonVisionSubsystem("Yellow_cam",
                PVConstants.D_ROBOT_TO_CAMERA, drive::addVisionMeasurement);

        gameHelpers = new GameHelpers(drive::getPose, drive::getRobotVelocity);
        flywheel = new FlywheelSubsystem(gameHelpers::getShotParameters);
        turret = new TurretSubsystem(drive::getPose, gameHelpers::getVirtualTargetFieldAngle);

        configurePathPlannerCommands();

        drive.resetOdometry(new Pose2d(1, 1, Rotation2d.kZero));
        // Coast during setup so the robot can be pushed into position
        setMotorBrake(false);
        driveFieldOrientedAngularVelocity = drive.driveCommand(
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                driverController.b(),
                () -> new Pose2d(gameHelpers.getVirtualTargetTranslation(), Rotation2d.kZero));
        drive.setDefaultCommand(driveFieldOrientedAngularVelocity);

        // Signal readiness to the driver so they know when to shoot
        leds.configureLEDs(() -> turret.isAutoTrackingEnabled()
                && turret.isOnTarget()
                && gameHelpers.isValidShotDistance());

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        // SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.addDefaultOption("None", Commands.none());
        autoChooser.addOption("Shoot in Place",
                Commands.parallel(
                        intake.lower().andThen(shoot()),
                        turret.autoTrack())
                        .withTimeout(6.0)
                        .andThen(Commands.sequence(
                                flywheel.stop(), indexer.stop(), intake.stop(),
                                turret.stopAutoTracking())));
        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
                .whileTrue(drive.lockWheels());
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
        drive.setMotorBrake(brake);
    }

    /**
     * Wraps an auto-driving command to prevent heading-lock snap-back.
     * <p>
     * The drive input stream caches the last heading. Without this wrapper,
     * the robot snaps to the pre-auto heading when the command ends because
     * the heading lock was never updated during auto driving.
     */
    private Command autoDriving(Command drivingCommand) {
        return drivingCommand.finallyDo(interrupted -> drive.stop()).withName("AutoDriving");
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
        Command auton = autoChooser.get();

        if (auton != null) {
            Logger.recordOutput("Robot/AutoSelected", auton.getName());
            return autoDriving(auton);
        } else {
            DriverStation.reportError(
                    "Auton error: No auton selected :(",
                    false);
            return Commands.none();
        }
    }
}
