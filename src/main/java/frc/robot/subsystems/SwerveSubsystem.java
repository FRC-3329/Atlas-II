package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import frc.robot.constants.Constants;
import frc.robot.constants.PathingConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Filesystem;

import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;

import dev.doglog.DogLog;

public class SwerveSubsystem extends SubsystemBase {
	private File directory = new File(Filesystem.getDeployDirectory(), "swerve");
	private SwerveDrive swerveDrive;
	private FollowPath.Builder pathBuilder;

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

		setupBLine();
	}

	@Override
	public void periodic() {
		DogLog.log(getName() + "/SwerveModuleStates", swerveDrive.getStates());
		DogLog.log(getName() + "/Pose", getPose());
		DogLog.log(getName() + "/RobotVelocity", getRobotVelocity());
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
		DogLog.time("Timing/Vision/AddMeasurementSeconds");
		swerveDrive.addVisionMeasurement(visionMeasurement.toPose2d(), timestampSeconds, stdDevs);
		DogLog.timeEnd("Timing/Vision/AddMeasurementSeconds");
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

	public void setupBLine() {
		pathBuilder = new FollowPath.Builder(
				this,
				swerveDrive::getPose,
				swerveDrive::getRobotVelocity,
				swerveDrive::drive,
				new PIDController(
						PathingConstants.TRANSLATION_kP,
						PathingConstants.TRANSLATION_kI,
						PathingConstants.TRANSLATION_kD),
				new PIDController(
						PathingConstants.ROTATION_kP,
						PathingConstants.ROTATION_kI,
						PathingConstants.ROTATION_kD),
				new PIDController(
						PathingConstants.CROSS_TRACK_kP,
						PathingConstants.CROSS_TRACK_kI,
						PathingConstants.CROSS_TRACK_kD))
				// BLine paths are authored on the blue side and flipped for the red alliance
				.withDefaultShouldFlip();
	}

	/** Loads a path from deploy/autos/paths and builds a command that follows it. */
	public Command followPath(String pathName) {
		try {
			return pathBuilder.build(new Path(pathName));
		} catch (Exception e) {
			DriverStation.reportError(
					"Unable to load path: " + pathName, e.getStackTrace());

			return Commands.none();
		}
	}
}
