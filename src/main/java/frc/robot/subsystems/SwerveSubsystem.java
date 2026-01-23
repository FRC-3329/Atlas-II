package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import static edu.wpi.first.units.Units.Meter;

/**
 * Subsystem for swerve drivetrain
 */
public class SwerveSubsystem extends SubsystemBase {
	// Configuration directory
	private File directory = new File(Filesystem.getDeployDirectory(), "swerve");
	private SwerveDrive swerveDrive;

	/**
	 * Creates a new swerve subsystem.
	 * 
	 * Loads the swerve configuration from JSON files in the deploy directory.
	 * Sets up the drivetrain with our max speed, and configures feedforward values for accurate motor control.
	 */
	public SwerveSubsystem() {
		// Initialize the swerve drive from config files
		try {
			// SwerveParser reads all the JSON config files and creates a SwerveDrive object
			// We give it our max speed and starting position (0, 0) at 0 degrees
			swerveDrive = new SwerveParser(directory)
				.createSwerveDrive(Constants.maxSpeed,
					// A pose is a position and rotation in 2D space
					new Pose2d(
						// X and Y position in meters
						new Translation2d(
							Meter.of(0), 
							Meter.of(0)
						),
						// Rotation
						Rotation2d.fromDegrees(0)
					)
				);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}

		/* 
			Chassis discretization helps make the swerve drive more accurate by accounting
			for the time delay between when we calculate speeds and when they're actually applied.
			0.02 = 20ms, which is our loop time (50Hz)
		*/
		swerveDrive.setChassisDiscretization(true, 0.02);
		
		/* 
			Replace the default feedforward with values we got from characterization	
			These numbers (kS, kV, kA) describe how our motors respond to voltage:
				- kS = voltage to overcome static friction (0.0846525V)
				- kV = voltage per unit velocity (2.68855V per m/s)
				- kA = voltage per unit acceleration (0.2266775V per m/s^2)
		*/
		swerveDrive.replaceSwerveModuleFeedforward(
			new SimpleMotorFeedforward(0.0846525, 2.68855, 0.2266775)
		);
	}

	@Override
	public void periodic() {
		// This runs every 20ms during all robot modes (teleop, auto, disabled)
	}

	@Override
	public void simulationPeriodic() {
		// This would run during simulation mode for testing without hardware
	}

	/**
	 * Gets the YAGSL {@link SwerveDrive} object.
	 * 
	 * This is useful when you need direct access to YAGSL methods that aren't wrapped by this subsystem. 
	 * For example, RobotContainer uses this to create the SwerveInputStream for teleop driving.
	 * 
	 * @return this subsystem's {@link SwerveDrive}
	 */
	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	/**
	 * Adds a vision measurement to improve the robot's position estimate.
	 * 
	 * This lets us use cameras (like PhotonVision) to correct our position estimate by looking at AprilTags. 
	 * The standard deviations (stdDevs) tell the Kalman filter how much to trust this measurement vs our wheel odometry.
	 * 
	 * @param visionMeasurement the pose measured by the camera
	 * @param timestampSeconds  when the measurement was taken (from FPGA timestamp)
	 * @param stdDevs           how much we trust this measurement (lower = more trust)
	 * 
	 * See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
	 */
	public void addVisionMeasurement(
		Pose2d visionMeasurement, 
		double timestampSeconds, 
		Matrix<N3, N1> stdDevs
	){
		swerveDrive.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
	}

	/**
	 * Drives the swerve drive field-oriented.
	 * 
	 * Field-oriented means "forward" on the joystick always moves the robot away 
	 * from the driver station, regardless of which way the robot is facing.
	 * 
	 * @param velocity the desired field-oriented {@link ChassisSpeeds} (vx, vy, omega)
	 */
	public void driveFieldOriented(ChassisSpeeds velocity) {
		swerveDrive.driveFieldOriented(velocity);
	}

	/**
	 * Command to drive the robot field-oriented.
	 * 
	 * The supplier is called every loop to get the latest desired speeds. 
	 * This is what we use for teleop driving with the SwerveInputStream.
	 * 
	 * @param velocity a {@link Supplier} that provides {@link ChassisSpeeds} every loop
	 * @return a command that drives the robot
	 */
	public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
		return run(() -> {
			swerveDrive.driveFieldOriented(velocity.get());
		});
	}

	/**
	 * Alternative drive command using heading control instead of angular velocity.
	 * 
	 * This version lets you specify a target heading direction (headingX, headingY) and
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
		DoubleSupplier headingY
	) {
		return run(() -> {
			// Scale translation inputs to 80%
			Translation2d scaledInputs = SwerveMath.scaleTranslation(
				new Translation2d(
					translationX.getAsDouble(), 
					translationY.getAsDouble()
				), 
				0.8
			);
			
			/*
				Calculate target speeds using YAGSL's heading controller
				This will automatically rotate the robot to face the heading direction
			*/ 
			driveFieldOriented(
				swerveDrive.swerveController.getTargetSpeeds(
					scaledInputs.getX(), 
					scaledInputs.getY(),
					headingX.getAsDouble(), 
					headingY.getAsDouble(), 
					swerveDrive.getOdometryHeading().getRadians(),
					swerveDrive.getMaximumChassisVelocity()
				)
			);
		});
	}

	/**
	 * Sets whether the drive motors should brake or coast when idle.
	 * 
	 * Brake mode: Motors actively resist movement when not powered.
	 *   - Good for precise control and stopping quickly
	 *   - Use during matches
	 * 
	 * Coast mode: Motors spin freely when not powered.
	 *   - Makes robot easier to push around manually
	 *   - Use during setup/testing
	 * 
	 * @param brake {@code true} for brake mode, {@code false} for coast mode
	 */
	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	/**
	 * Command to zero the gyro (reset heading to 0 degrees).
	 * 
	 * This sets the current direction the robot is facing as "forward" (0 degrees).
	 * Waits 0.5s after zeroing to let the gyro settle before continuing.
	 * 
	 * @return command to zero the gyro and wait
	 */
	public Command zeroGyro() {
		return Commands
			.runOnce(() -> swerveDrive.zeroGyro())
			.andThen(Commands.waitSeconds(0.5));
	}

	/**
	 * Gets the current gyro heading.
	 * 
	 * @return the gyro's yaw (rotation around vertical axis)
	 */
	public Rotation2d getGyro() {
		return swerveDrive.getYaw();
	}

	/**
	 * Resets the robot's position on the field.
	 * 
	 * @param pose the worldspace position to set the robot to
	 */
	public void resetOdometry(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}

	/**
	 * Command for characterizing the swerve module angle motors.
	 * 
	 * SysId (System Identification) runs automated tests to measure how the motors respond to voltage. 
	 * This helps us tune PID controllers and feedforward values for better control.
	 * 
	 * This version tests the angle (steering) motors specifically.
	 * 
	 * @return angle motor characterization command
	 */
	public Command getAngleCharacterizationCommand() {
		return SwerveDriveTest.generateSysIdCommand(
			SwerveDriveTest.setAngleSysIdRoutine(
				new SysIdRoutine.Config(), this, swerveDrive
			),
			3, 6, 3
		);
	}

	/**
	 * Command for characterizing the swerve module drive motors.
	 * 
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
				12, true
			),
			3, 4, 1.5
		);
	}
}
