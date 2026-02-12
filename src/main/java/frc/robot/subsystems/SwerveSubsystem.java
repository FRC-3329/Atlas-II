package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import frc.robot.constants.Constants;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

		/*
		 * Chassis discretization helps make the swerve drive more accurate by
		 * accounting
		 * for the time delay between when we calculate speeds and when they're actually
		 * applied.
		 */
		swerveDrive.setChassisDiscretization(true, 0.02);

		/*
		 * kS = voltage to overcome static friction (0.0846525V)
		 * kV = voltage per unit velocity (2.68855V per m/s)
		 * kA = voltage per unit acceleration (0.2266775V per m/s^2)
		 */
		swerveDrive.replaceSwerveModuleFeedforward(
				new SimpleMotorFeedforward(0.0846525, 2.68855, 0.2266775));

		setupPathPlanner();
	}

	@Override
	public void periodic() {
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
	 * This lets us use cameras (like PV or QN) to correct our position estimate by
	 * looking at AprilTags.
	 * The standard deviations (stdDevs) tell the Kalman filter how much to trust
	 * this measurement vs our wheel odometry.
	 * 
	 * @param visionMeasurement the pose measured by the camera (will be converted
	 *                          from 3D to 2D)
	 * @param timestampSeconds  when the measurement was taken (from FPGA timestamp)
	 * @param stdDevs           how much we trust this measurement (lower = more
	 *                          trust)
	 * 
	 *                          See
	 *                          {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
	 */
	public void addVisionMeasurement(
			Pose3d visionMeasurement,
			double timestampSeconds,
			Matrix<N3, N1> stdDevs) {
		swerveDrive.addVisionMeasurement(visionMeasurement.toPose2d(), timestampSeconds, stdDevs);
	}

	/**
	 * Field-oriented means "forward" on the joystick always moves the robot away
	 * from the driver station, regardless of which way the robot is facing.
	 * 
	 * @param velocity the desired field-oriented {@link ChassisSpeeds} (vx, vy,
	 *                 omega)
	 */
	public void driveFieldOriented(ChassisSpeeds velocity) {
		swerveDrive.driveFieldOriented(velocity);
	}

	/**
	 * @param velocity a {@link Supplier} that provides {@link ChassisSpeeds} every
	 *                 loop
	 * @return a command that drives the robot
	 */
	public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
		return run(() -> {
			swerveDrive.driveFieldOriented(velocity.get());
		}).withName("SwerveDriveFieldOriented");
	}

	/**
	 * This version lets you specify a target heading direction (headingX, headingY)
	 * and
	 * the robot will automatically rotate to face that direction while translating.
	 * 
	 * @param translationX forward/backward speed (-1 to 1)
	 * @param translationY left/right speed (-1 to 1)
	 * @param headingX     target heading X component (like right stick X)
	 * @param headingY     target heading Y component (like right stick Y)
	 * @return command that drives with heading control
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

	/**
	 * @param brake {@code true} for brake mode, {@code false} for coast mode
	 */
	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	public Command zeroGyro() {
		return Commands
				.runOnce(() -> swerveDrive.zeroGyro())
				.andThen(Commands.waitSeconds(0.5))
				.withName("SwerveZeroGyro");
	}

	/**
	 * Lock the wheels in an X formation for defense
	 * This makes the robot very difficult to push
	 * 
	 * @return command to lock wheels
	 */
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

	/**
	 * SysId runs automated tests to measure how the motors respond to voltage.
	 * This helps us tune PID controllers and feedforward values for better control.
	 * 
	 * This version tests the angle (steering) motors specifically.
	 * 
	 * @return angle motor characterization command
	 */
	public Command getAngleCharacterizationCommand() {
		return SwerveDriveTest.generateSysIdCommand(
				SwerveDriveTest.setAngleSysIdRoutine(
						new SysIdRoutine.Config(), this, swerveDrive),
				3, 6, 3);
	}

	/**
	 * SysId runs automated tests to measure how the motors respond to voltage.
	 * This helps us tune PID controllers and feedforward values for better control.
	 * 
	 * This version tests the drive (wheel) motors specifically.
	 * 
	 * @return drive motor characterization command
	 */
	public Command getDriveCharacterizationCommand() {
		return SwerveDriveTest.generateSysIdCommand(
				SwerveDriveTest.setDriveSysIdRoutine(
						new SysIdRoutine.Config(), this, swerveDrive,
						12, true),
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

					// TODO: PID needs to be tuned
					new PPHolonomicDriveController(
							new PIDConstants(3.0, 0.0, 0.1), // Translation
							new PIDConstants(3.0, 0.0, 0.1) // Rotation
					),
					config,
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

	/**
	 * @param pathName name of the PathPlanner auto
	 * @return command to follow the autonomous path
	 */
	public Command getAutonomousCommand(String pathName) {
		return new PathPlannerAuto(pathName);
	}

	/**
	 * @param pathName    name of the path to follow
	 * @param constraints constraints for pathfinding
	 * @return command to pathfind and follow
	 */
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
