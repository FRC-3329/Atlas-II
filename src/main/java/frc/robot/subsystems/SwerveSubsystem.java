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

public class SwerveSubsystem extends SubsystemBase {
	private File directory = new File(Filesystem.getDeployDirectory(), "swerve");
	private SwerveDrive swerveDrive;

	/**
	 * Creates a new swerve subsystem.
	 */
	public SwerveSubsystem() {
		// Initialize swerve drive
		try {
			swerveDrive = new SwerveParser(directory)
				.createSwerveDrive(Constants.maxSpeed,
					new Pose2d(
						new Translation2d(Meter.of(0), Meter.of(0)),
						Rotation2d.fromDegrees(0)
					)
				);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}

		swerveDrive.setChassisDiscretization(true, 0.02);
		swerveDrive.replaceSwerveModuleFeedforward(
			new SimpleMotorFeedforward(0.0846525, 2.68855, 0.2266775)
		);
	}

	@Override
	public void periodic() {
	}

	@Override
	public void simulationPeriodic() {
	}

	/**
	 * Gets the YAGSL {@link SwerveDrive} in this subsystem.
	 * 
	 * @return this subsystem's {@link SwerveDrive}
	 */
	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	/**
	 * See
	 * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
	 */
	public void addVisionMeasurement(
		Pose2d visionMeasurement, 
		double timestampSeconds, 
		Matrix<N3, N1> stdDevs
	){
		swerveDrive.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
	}

	/**
	 * Drives the swervedrive field oriented.
	 * 
	 * @param velocity the field oriented {@link ChassisSpeeds}
	 */
	public void driveFieldOriented(ChassisSpeeds velocity) {
		swerveDrive.driveFieldOriented(velocity);
	}

	/**
	 * Command to drive the robot field oriented.
	 * 
	 * @param velocity a {@link ChassisSpeeds} {@link Supplier} to provide data
	 * @return
	 */
	public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
		return run(() -> {
			swerveDrive.driveFieldOriented(velocity.get());
		});
	}

	/**
	 * Another drive command
	 * 
	 * @param translationX
	 * @param translationY
	 * @param headingX
	 * @param headingY
	 * @return
	 */
	public Command driveCommand(
		DoubleSupplier translationX, 
		DoubleSupplier translationY, 
		DoubleSupplier headingX,
		DoubleSupplier headingY
	) {
		return run(() -> {
			Translation2d scaledInputs = SwerveMath.scaleTranslation(
				new Translation2d(
					translationX.getAsDouble(), 
					translationY.getAsDouble()
				), 
				0.8
			);
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
	 * Whether the drivetrain should be in brake or coast mode.
	 * 
	 * @param brake {@code true} is brake, {@code false} is coast
	 */
	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	/**
	 * Command to zero the gyro. Will wait 0.5s after zeroing.
	 * 
	 * @return command to zero the gyro
	 */
	public Command zeroGyro() {
		return Commands
			.runOnce(() -> swerveDrive.zeroGyro())
			.andThen(Commands.waitSeconds(0.5));
	}

	/**
	 * Gets the gyro's yaw.
	 * 
	 * @return the gyro's yaw
	 */
	public Rotation2d getGyro() {
		return swerveDrive.getYaw();
	}

	/**
	 * Reset's both the vision and encoder/gyro odometry positions.
	 * <p>
	 * <b>note: don't set it to (0, 0).</b> for some reason the Quest does not reset
	 * the position when commanded to (0, 0).
	 * 
	 * @param pose the worldspace positon to set the robot.
	 */
	public void resetOdometry(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}

	/**
	 * @return angle motor characterization command
	 */
	public Command getAngleCharacterizationCommand() {
		return SwerveDriveTest.generateSysIdCommand(
			SwerveDriveTest.setAngleSysIdRoutine(
				new SysIdRoutine.Config(), 
				this, 
				swerveDrive
			),
			3,
			6,
			3
		);
	}

	/**
	 * @return drive motor characterization command
	 */
	public Command getDriveCharacterizationCommand() {
		return SwerveDriveTest.generateSysIdCommand(
			SwerveDriveTest.setDriveSysIdRoutine(
				new SysIdRoutine.Config(), 
				this, 
				swerveDrive, 
				12, 
				true
			),
			3,
			4,
			1.5
		);
	}
}
