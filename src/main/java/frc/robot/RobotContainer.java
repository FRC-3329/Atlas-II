package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CalibrateQuestCommand;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import swervelib.SwerveInputStream;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    private final QuestNavSubsystem questNavSystem = new QuestNavSubsystem(
        drivebase::addVisionMeasurement, 
        true
    );
    private final PhotonVisionSubsystem photonVisionSubsystem = new PhotonVisionSubsystem(
        drivebase::addVisionMeasurement, 
        false
    );

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(
        OperatorConstants.kDriverControllerPort
    );
    private final CommandXboxController operatorController = new CommandXboxController(
        OperatorConstants.kOperatorControllerPort
    );

    // Commands
    private final SwerveInputStream driveAngularVelocity;
    private final Command driveFieldOrientedAngularVelocity;

    private boolean autoDriving = false;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        resetOdometry(new Pose2d(1, 1, Rotation2d.kZero));

        /*
            On teleop, reset the odometry to the PV result only if not connected to FMS.
		    At comp, we only want to reset the pose at the start of auton because we know
		    the robot will be looking at a tag. 
            We cannot garentee that at the start ofteleop.
        */
        new Trigger(RobotState::isTeleop)
            .and(
                new Trigger(DriverStation::isFMSAttached).negate()
            ).onTrue(autoDriving(
                Commands.runOnce(() -> {
                    photonVisionSubsystem.getPoseOptional().ifPresent(pose -> {
                        resetOdometry(pose);  
                    });
                })
            ));

		/*
		 * In the event that QN stops tracking, 
         * failover to PV for updating our global robot pose. 
         * This will be accurate but require a tag in sight so it may
		 * become inaccurate if the robot cannot see any tags.
		 */
        new Trigger(questNavSystem::isTracking)
            .debounce(0.5, DebounceType.kFalling)
            .onFalse(Commands.runOnce(() -> {
                DriverStation.reportError(
                    "QN tracking lost!! Failing over to PV...", 
                    false
                );

                questNavSystem.useEstimatedConsumer(false);
                photonVisionSubsystem.useEstimatedConsumer(true);
            }));

        // Configure motor brake mode (false = coast)
        setMotorBrake(false);

        // Configure drive input stream with deadband and alliance-relative control
        driveAngularVelocity = SwerveInputStream
                .of(drivebase.getSwerveDrive(), 
                    () -> -driverController.getLeftY(), 
                    () -> -driverController.getLeftX()
                )
                .withControllerRotationAxis(
                    () -> -driverController.getRightX()
                )
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);
        driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.setDefaultOption("None", Commands.none());

        configureBindings();
    }

    /**
	 * Resets both the drivetrain's position and sets the quest's position.
	 * 
	 * @param pose the position in the world
	 */
    private void resetOdometry(Pose2d pose) {
        questNavSystem.setQuestPose(pose);
        drivebase.resetOdometry(pose);
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
            });
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

        // Callibrate QN when holding B on driver controller
        driverController.b().whileTrue(
            autoDriving(
                new CalibrateQuestCommand(drivebase, questNavSystem)
            )
        );

		// Add button to the dashboard to reset QN's position with PV
		SmartDashboard.putData(
            "Reset QN from PV",
            Commands.runOnce(() -> {
			    DataLogManager.log("Attempting to reset QN to PV pose...");

			    photonVisionSubsystem.getPoseOptional().ifPresent(pose -> {
				    photonVisionSubsystem.useEstimatedConsumer(false);
				    questNavSystem.setQuestPose(pose);
				    questNavSystem.useEstimatedConsumer(true);
				    DataLogManager.log("QN pose successfully set from PV!");
			    });
		    }
        ));
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
            // Set internal state to tracking
            autoDriving = true;
            // Update state in controller supplier
            driveAngularVelocity.get();
        }).finallyDo(interrupted -> {
            autoDriving = false;
            driveAngularVelocity.get();
        });
    }

    public Command getAutonomousCommand() {
        Command auton = autoChooser.getSelected();
        Optional<Pose2d> pvPose = photonVisionSubsystem.getPoseOptional();

        // auton selected and valid PV pose
        if (auton != null && pvPose.isPresent()) {
            return autoDriving(
                auton.beforeStarting(
                    () -> resetOdometry(pvPose.get())
                )
            );
        // auton selected and no valid PV pose
        } else if (auton != null && pvPose.isEmpty()) {
            DriverStation.reportError(
                "Auton error: Auton selected but the PV pose is invalid :(", 
                false
            );

            return Commands.none();
        // no auton selected and valid PV pose
        } else if (auton == null && pvPose.isPresent()) {
            DriverStation.reportError(
                "Auton error: Auton not selected (PV pose is valid)", 
                false
            );

            return Commands.runOnce(() -> {
                resetOdometry(pvPose.get());
            });
        } else {
            DriverStation.reportError(
                "Auton error: Auton not selected and PV pose is invalid :(", 
                false
            );

            return Commands.none();
        }
    }
}
