package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOSparkMax;
import frc.robot.utils.AKTimeLogger;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    private static final int FRONT_LEFT = 0;
    private static final int FRONT_RIGHT = 1;
    private static final int BACK_LEFT = 2;
    private static final int BACK_RIGHT = 3;

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final PIDController headingController = new PIDController(5.0, 0.0, 0.1);

    private ChassisSpeeds robotRelativeSetpoint = new ChassisSpeeds();
    private Rotation2d rawGyroRotation = Rotation2d.kZero;

    public Swerve() {
        switch (Constants.currentMode) {
            case REAL:
                gyroIO = new GyroIOPigeon2();

                modules = new Module[] {
                        new Module("FrontLeft", new ModuleIOSparkMax(
                                SwerveConstants.FrontLeftModule.DRIVE_ID,
                                SwerveConstants.FrontLeftModule.ANGLE_ID,
                                SwerveConstants.FrontLeftModule.ENCODER_ID,
                                SwerveConstants.FrontLeftModule.ABSOLUTE_ENCODER_OFFSET_DEGREES,
                                SwerveConstants.FrontLeftModule.DRIVE_INVERTED,
                                SwerveConstants.FrontLeftModule.ANGLE_INVERTED,
                                SwerveConstants.FrontLeftModule.ENCODER_INVERTED)),
                        new Module("FrontRight", new ModuleIOSparkMax(
                                SwerveConstants.FrontRightModule.DRIVE_ID,
                                SwerveConstants.FrontRightModule.ANGLE_ID,
                                SwerveConstants.FrontRightModule.ENCODER_ID,
                                SwerveConstants.FrontRightModule.ABSOLUTE_ENCODER_OFFSET_DEGREES,
                                SwerveConstants.FrontRightModule.DRIVE_INVERTED,
                                SwerveConstants.FrontRightModule.ANGLE_INVERTED,
                                SwerveConstants.FrontRightModule.ENCODER_INVERTED)),
                        new Module("BackLeft", new ModuleIOSparkMax(
                                SwerveConstants.BackLeftModule.DRIVE_ID,
                                SwerveConstants.BackLeftModule.ANGLE_ID,
                                SwerveConstants.BackLeftModule.ENCODER_ID,
                                SwerveConstants.BackLeftModule.ABSOLUTE_ENCODER_OFFSET_DEGREES,
                                SwerveConstants.BackLeftModule.DRIVE_INVERTED,
                                SwerveConstants.BackLeftModule.ANGLE_INVERTED,
                                SwerveConstants.BackLeftModule.ENCODER_INVERTED)),
                        new Module("BackRight", new ModuleIOSparkMax(
                                SwerveConstants.BackRightModule.DRIVE_ID,
                                SwerveConstants.BackRightModule.ANGLE_ID,
                                SwerveConstants.BackRightModule.ENCODER_ID,
                                SwerveConstants.BackRightModule.ABSOLUTE_ENCODER_OFFSET_DEGREES,
                                SwerveConstants.BackRightModule.DRIVE_INVERTED,
                                SwerveConstants.BackRightModule.ANGLE_INVERTED,
                                SwerveConstants.BackRightModule.ENCODER_INVERTED))
                };

                break;
            case SIM:
                gyroIO = new GyroIOSim();

                modules = new Module[] {
                        new Module("FrontLeft", new ModuleIOSim()),
                        new Module("FrontRight", new ModuleIOSim()),
                        new Module("BackLeft", new ModuleIOSim()),
                        new Module("BackRight", new ModuleIOSim())
                };

                break;
            case REPLAY:
            default:
                gyroIO = new GyroIO() {
                };

                modules = new Module[] {
                        new Module("FrontLeft", new ModuleIO() {
                        }),
                        new Module("FrontRight", new ModuleIO() {
                        }),
                        new Module("BackLeft", new ModuleIO() {
                        }),
                        new Module("BackRight", new ModuleIO() {
                        })
                };

                break;
        }

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        poseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.KINEMATICS,
                rawGyroRotation,
                getModulePositions(),
                Pose2d.kZero);

        SmartDashboard.putData("ZeroGyro", zeroGyro().withName("Zero Gyro"));
        setupPathPlanner();
    }

    @Override
    public void periodic() {
        if (gyroIO instanceof GyroIOSim gyroSim) {
            rawGyroRotation = rawGyroRotation.plus(
                    Rotation2d.fromRadians(robotRelativeSetpoint.omegaRadiansPerSecond * Constants.LOOP_TIME_SECONDS));
            gyroSim.setYaw(rawGyroRotation.getRadians(), robotRelativeSetpoint.omegaRadiansPerSecond);
        }

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs(getName() + "/Gyro", gyroInputs);

        for (Module module : modules) {
            module.updateInputs();
        }

        rawGyroRotation = Rotation2d.fromRadians(gyroInputs.yawPositionRad);
        poseEstimator.update(rawGyroRotation, getModulePositions());

        Logger.recordOutput(getName() + "/Pose", getPose());
        Logger.recordOutput(getName() + "/RobotVelocity", getRobotVelocity());
        Logger.recordOutput(getName() + "/Setpoint", robotRelativeSetpoint);
        Logger.recordOutput(getName() + "/MeasuredStates", getModuleStates());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getRobotVelocity() {
        return SwerveConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public Rotation2d getGyro() {
        return rawGyroRotation;
    }

    public void addVisionMeasurement(
            Pose3d visionMeasurement,
            double timestampSeconds,
            Matrix<N3, N1> stdDevs) {
        AKTimeLogger.startTiming("Timing/Vision/AddMeasurementSeconds");
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), timestampSeconds, stdDevs);
        AKTimeLogger.endTiming("Timing/Vision/AddMeasurementSeconds");
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    public void driveFieldOriented(ChassisSpeeds fieldRelativeSpeeds) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds.vxMetersPerSecond,
                fieldRelativeSpeeds.vyMetersPerSecond,
                fieldRelativeSpeeds.omegaRadiansPerSecond,
                getPose().getRotation());

        driveRobotRelative(speeds);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
        return run(() -> driveFieldOriented(fieldRelativeSpeeds.get()))
                .withName("SwerveDriveFieldOriented");
    }

    public Command driveCommand(
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier rotationX,
            BooleanSupplier aim,
            Supplier<Pose2d> aimTarget) {
        return run(() -> {
            double x = modifyAxis(translationX.getAsDouble()) * SwerveConstants.MAX_LINEAR_SPEED_METERS_PER_SEC;
            double y = modifyAxis(translationY.getAsDouble()) * SwerveConstants.MAX_LINEAR_SPEED_METERS_PER_SEC;

            Translation2d scaledTranslation = new Translation2d(x, y).times(0.8);

            if (shouldFlipForAlliance()) {
                scaledTranslation = scaledTranslation.times(-1.0);
            }

            double omega;
            if (aim.getAsBoolean()) {
                Translation2d toTarget = aimTarget.get().getTranslation().minus(getPose().getTranslation());
                Rotation2d targetHeading = toTarget.getAngle().plus(Rotation2d.fromDegrees(180.0));

                omega = headingController.calculate(getPose().getRotation().getRadians(), targetHeading.getRadians());
            } else {
                omega = -modifyAxis(rotationX.getAsDouble()) * SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC;
            }

            driveFieldOriented(new ChassisSpeeds(scaledTranslation.getX(), scaledTranslation.getY(), omega));
        }).withName("SwerveDriveTeleop");
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        robotRelativeSetpoint = ChassisSpeeds.discretize(speeds, Constants.LOOP_TIME_SECONDS);
        SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(robotRelativeSetpoint);

        setModuleStates(states);
    }

    public void setMotorBrake(boolean brake) {
        for (Module module : modules) {
            module.setBrakeMode(brake);
        }
    }

    public Command zeroGyro() {
        return Commands.runOnce(() -> {
            gyroIO.zeroGyro();
            rawGyroRotation = Rotation2d.kZero;
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.kZero));
        }, this).withName("SwerveZeroGyro");
    }

    public Command lockWheels() {
        return run(() -> setModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(0.0, SwerveConstants.FRONT_LEFT_LOCATION.getAngle()),
                new SwerveModuleState(0.0, SwerveConstants.FRONT_RIGHT_LOCATION.getAngle()),
                new SwerveModuleState(0.0, SwerveConstants.BACK_LEFT_LOCATION.getAngle()),
                new SwerveModuleState(0.0, SwerveConstants.BACK_RIGHT_LOCATION.getAngle())
        })).withName("SwerveLockWheels");
    }

    public void stop() {
        robotRelativeSetpoint = new ChassisSpeeds();

        for (Module module : modules) {
            module.stop();
        }
    }

    public void setupPathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotVelocity,
                    (speedsRobotRelative, moduleFeedForwards) -> driveRobotRelative(speedsRobotRelative),
                    new PPHolonomicDriveController(
                            new PIDConstants(4.5, 0.0, 0.2),
                            new PIDConstants(4.5, 0.0, 0.2)),
                    config,
                    () -> shouldFlipForAlliance(),
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

    private void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_LINEAR_SPEED_METERS_PER_SEC);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                modules[FRONT_LEFT].getPosition(),
                modules[FRONT_RIGHT].getPosition(),
                modules[BACK_LEFT].getPosition(),
                modules[BACK_RIGHT].getPosition()
        };
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                modules[FRONT_LEFT].getState(),
                modules[FRONT_RIGHT].getState(),
                modules[BACK_LEFT].getState(),
                modules[BACK_RIGHT].getState()
        };
    }

    private static double modifyAxis(double value) {
        double deadbanded = Math.abs(value) > OperatorConstants.DEADBAND ? value : 0.0;
        return Math.copySign(deadbanded * deadbanded, deadbanded);
    }

    private static boolean shouldFlipForAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    private static class Module {
        private final String name;
        private final ModuleIO io;
        private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

        Module(String name, ModuleIO io) {
            this.name = name;
            this.io = io;
        }

        void updateInputs() {
            io.updateInputs(inputs);
            Logger.processInputs("Swerve/Module" + name, inputs);
        }

        SwerveModulePosition getPosition() {
            return new SwerveModulePosition(
                    inputs.drivePositionMeters,
                    Rotation2d.fromRadians(inputs.anglePositionRad));
        }

        SwerveModuleState getState() {
            return new SwerveModuleState(
                    inputs.driveVelocityMetersPerSec,
                    Rotation2d.fromRadians(inputs.anglePositionRad));
        }

        void setDesiredState(SwerveModuleState state) {
            SwerveModuleState optimizedState = optimize(state, Rotation2d.fromRadians(inputs.anglePositionRad));
            io.setDesiredState(optimizedState);
        }

        void setBrakeMode(boolean brake) {
            io.setBrakeMode(brake);
        }

        void stop() {
            io.stop();
        }

        private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
            Rotation2d delta = desiredState.angle.minus(currentAngle);

            if (Math.abs(delta.getDegrees()) > 90.0) {
                return new SwerveModuleState(
                        -desiredState.speedMetersPerSecond,
                        desiredState.angle.plus(Rotation2d.fromDegrees(180.0)));
            }

            return desiredState;
        }
    }
}
