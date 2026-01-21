package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
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
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Meter;

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
	private Command pathfindingCommand;

	/**
	 * Creates a new swerve subsystem.
	 * 
	 * @param resetVision        A consumer that accepts a {@link Pose2d} to reset
	 *                           the vision system's position when requested
	 * @param visionDataSupplier A supplier for giving {@link VisionData} to the
	 *                           drivebase.
	 */
	public SwerveSubsystem() {
		// Initalize swerve drives
		try {
			swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed,
					new Pose2d(new Translation2d(Meter.of(0), Meter.of(0)), Rotation2d.fromDegrees(0)));
		} catch (Exception e) {
			throw new RuntimeException(e);
		}

		swerveDrive.setChassisDiscretization(true, 0.02);
		swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(0.0846525, 2.68855, 0.2266775));

		setupPathPlanner();

		// Auto pathing
		// Load the path we want to pathfind to and follow
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile("Auto Path");
			// Create the constraints to use while pathfinding. The constraints defined in
			// the path will only be used for the path.
			PathConstraints constraints = new PathConstraints(
					2.0, 1.0,
					Units.degreesToRadians(720), Units.degreesToRadians(360));

			// Since AutoBuilder is configured, we can use it to build pathfinding commands
			pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
					path,
					constraints);
		} catch (Exception e) {
			DriverStation.reportError("Unable to load autonomus path for path following!!", e.getStackTrace());
			pathfindingCommand = Commands.none();
		}
	}

	/**
	 * Gets the path following command. Probably will need to replace this with
	 * something more robust when adding more than one path.
	 * 
	 * @return The command to run this path.
	 */
	public Command getPathFindingCommand() {
		return pathfindingCommand.finallyDo(() -> swerveDrive.drive(new ChassisSpeeds(0, 0, 0)));
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
			Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
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
	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
			DoubleSupplier headingY) {
		return run(() -> {
			Translation2d scaledInputs = SwerveMath.scaleTranslation(
					new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);
			driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
					headingX.getAsDouble(), headingY.getAsDouble(), swerveDrive.getOdometryHeading().getRadians(),
					swerveDrive.getMaximumChassisVelocity()));
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
		return Commands.runOnce(() -> swerveDrive.zeroGyro()).andThen(Commands.waitSeconds(0.5));
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
				SwerveDriveTest.setAngleSysIdRoutine(new SysIdRoutine.Config(), this, swerveDrive),
				3,
				6,
				3);
	}

	/**
	 * @return drive motor characterization command
	 */
	public Command getDriveCharacterizationCommand() {
		return SwerveDriveTest.generateSysIdCommand(
				SwerveDriveTest.setDriveSysIdRoutine(new SysIdRoutine.Config(), this, swerveDrive, 12, true),
				3,
				4,
				1.5);
	}

	public void setupPathPlanner() {
		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();
			final boolean enableFeedforward = true;
			AutoBuilder.configure(swerveDrive::getPose, swerveDrive::resetOdometry, swerveDrive::getRobotVelocity,
					(speedsRobotRelative, moduleFeedForwards) -> {
						if (enableFeedforward) {
							swerveDrive.drive(speedsRobotRelative,
									swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
									moduleFeedForwards.linearForces());
						} else {
							swerveDrive.setChassisSpeeds(speedsRobotRelative);
						}
					}, new PPHolonomicDriveController(new PIDConstants(3.0, 0.0, 0.1), new PIDConstants(3.0, 0.0, 0.1)),
					config,
					() -> {
						var alliance = DriverStation.getAlliance();
						if (alliance.isPresent()) {
							return alliance.get() == DriverStation.Alliance.Red;
						}
						return false;
					}, this);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Gets the path by name.
	 * 
	 * @param pathName name of the path
	 * @return the command to follow the path
	 */
	public Command getAutonomousCommand(String pathName) {
		return new PathPlannerAuto(pathName);
	}
}
