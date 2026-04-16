package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import frc.robot.constants.Constants;
import frc.robot.utils.AKTimeLogger;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.Matrix;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Filesystem;

import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
	private File directory = new File(Filesystem.getDeployDirectory(), "swerve");
	private SwerveDrive swerveDrive;

	public SwerveSubsystem() {
		try {
			swerveDrive = new SwerveParser(directory)
					.createSwerveDrive(Constants.MAX_SPEED,
							new Pose2d(
									new Translation2d(
											Meter.of(0),
											Meter.of(0)),
									Rotation2d.fromDegrees(0)));
		} catch (Exception e) {
			throw new RuntimeException(e);
		}

		// Compensates for the delay between calculating and applying speeds,
		// which otherwise causes the robot to arc instead of driving straight.
		swerveDrive.setChassisDiscretization(true, 0.02);
		swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
		swerveDrive.setHeadingCorrection(true);

		SmartDashboard.putData("ZeroGyro", zeroGyro().withName("Zero Gyro"));

		setupPathPlanner();
	}

	@Override
	public void periodic() {
		Logger.recordOutput(getName() + "/SwerveModuleStates", swerveDrive.getStates());
		Logger.recordOutput(getName() + "/Pose", getPose());
		Logger.recordOutput(getName() + "/RobotVelocity", getRobotVelocity());
	}

	@Override
	public void simulationPeriodic() {
	}

	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	public ChassisSpeeds getRobotVelocity() {
		return swerveDrive.getRobotVelocity();
	}

	/**
	 * Fuses a vision pose measurement into the Kalman filter.
	 * Standard deviations control how much to trust vision vs. wheel odometry.
	 */
	public void addVisionMeasurement(
			Pose3d visionMeasurement,
			double timestampSeconds,
			Matrix<N3, N1> stdDevs) {
		AKTimeLogger.startTiming("Timing/Vision/AddMeasurementSeconds");
		swerveDrive.addVisionMeasurement(visionMeasurement.toPose2d(), timestampSeconds, stdDevs);
		AKTimeLogger.endTiming("Timing/Vision/AddMeasurementSeconds");
	}

	public void driveFieldOriented(ChassisSpeeds velocity) {
		swerveDrive.driveFieldOriented(velocity);
	}

	public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
		return run(() -> {
			swerveDrive.driveFieldOriented(velocity.get());
		}).withName("SwerveDriveFieldOriented");
	}

	/**
	 * Drives with a target heading direction via the right stick, allowing
	 * simultaneous translation and heading control.
	 */
	public Command driveCommand(
			DoubleSupplier translationX,
			DoubleSupplier translationY,
			DoubleSupplier headingX,
			DoubleSupplier headingY) {
		return run(() -> {
			Translation2d scaledInputs = SwerveMath.scaleTranslation(
					new Translation2d(
							translationX.getAsDouble(),
							translationY.getAsDouble()),
					0.8);

			driveFieldOriented(
					swerveDrive.swerveController.getTargetSpeeds(
							scaledInputs.getX(),
							scaledInputs.getY(),
							headingX.getAsDouble(),
							headingY.getAsDouble(),
							swerveDrive.getOdometryHeading().getRadians(),
							swerveDrive.getMaximumChassisVelocity()));
		}).withName("SwerveDriveWithHeading");
	}

	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	public Command zeroGyro() {
		return Commands
				.runOnce(() -> swerveDrive.zeroGyro())
				.andThen(Commands.waitSeconds(0.5))
				.withName("SwerveZeroGyro");
	}

	/** Locks the wheels in an X formation so the robot can't be pushed. */
	public Command lockWheels() {
		return run(() -> swerveDrive.lockPose())
				.withName("SwerveLockWheels");
	}

	public Rotation2d getGyro() {
		return swerveDrive.getYaw();
	}

	public void resetOdometry(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}

	/** SysId characterization for the angle (steering) motors. */
	public Command getAngleCharacterizationCommand() {
		return SwerveDriveTest.generateSysIdCommand(
				SwerveDriveTest.setAngleSysIdRoutine(
						new SysIdRoutine.Config(), this, swerveDrive),
				3, 6, 3);
	}

	/** SysId characterization for the drive (wheel) motors. */
	public Command getDriveCharacterizationCommand() {
		return SwerveDriveTest.generateSysIdCommand(
				SwerveDriveTest.setDriveSysIdRoutine(
						new SysIdRoutine.Config(), this, swerveDrive,
						12, false),
				3, 4, 1.5);
	}

	public void setupPathPlanner() {
		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();
			final boolean enableFeedforward = true;

			AutoBuilder.configure(
					swerveDrive::getPose, // Supplier for current robot pose
					swerveDrive::resetOdometry, // Consumer to reset odometry
					swerveDrive::getRobotVelocity, // Supplier for current robot velocity
					(speedsRobotRelative, moduleFeedForwards) -> {
						if (enableFeedforward) {
							swerveDrive.drive(
									speedsRobotRelative,
									swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
									moduleFeedForwards.linearForces());
						} else {
							swerveDrive.setChassisSpeeds(speedsRobotRelative);
						}
					},
					new PPHolonomicDriveController(
							new PIDConstants(4.5, 0.0, 0.2),
							new PIDConstants(4.5, 0.0, 0.2)
					),
					config,
					// PathPlanner needs to mirror paths for the red alliance
					() -> {
						Optional<Alliance> alliance = DriverStation.getAlliance();

						if (alliance.isPresent()) {
							return alliance.get() == DriverStation.Alliance.Red;
						}

						return false;
					},
					this);
		} catch (Exception e) {
			DriverStation.reportError(
					"Failed to setup PathPlanner: " + e.getMessage(), e.getStackTrace());
		}
	}

	public Command getAutonomousCommand(String pathName) {
		return new PathPlannerAuto(pathName);
	}

	public Command pathfindThenFollowPath(String pathName, PathConstraints constraints) {
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
			return AutoBuilder.pathfindThenFollowPath(path, constraints);
		} catch (Exception e) {
			DriverStation.reportError(
					"Unable to load path: " + pathName, e.getStackTrace());

			return Commands.none();
		}
	}
}
