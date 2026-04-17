package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

// Adapted from previous YAGSL config files
public class SwerveConstants {
    public static final double MAX_LINEAR_SPEED_METERS_PER_SEC = Constants.MAX_SPEED;
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 2.0 * Math.PI;

    public static final double DRIVE_WHEEL_RADIUS_METERS = Units.inchesToMeters(
            PhysicalProperties.DRIVE_WHEEL_DIAMETER_INCHES) / 2.0;
    public static final double DRIVE_POSITION_FACTOR_METERS_PER_ROTATION = 2.0 * Math.PI * DRIVE_WHEEL_RADIUS_METERS
            / PhysicalProperties.DRIVE_GEAR_RATIO;
    public static final double DRIVE_VELOCITY_FACTOR_METERS_PER_SEC_PER_RPM = DRIVE_POSITION_FACTOR_METERS_PER_ROTATION
            / 60.0;
    public static final double ANGLE_POSITION_FACTOR_RAD_PER_ROTATION = 2.0 * Math.PI
            / PhysicalProperties.ANGLE_GEAR_RATIO;
    public static final double ANGLE_VELOCITY_FACTOR_RAD_PER_SEC_PER_RPM = ANGLE_POSITION_FACTOR_RAD_PER_ROTATION
            / 60.0;

    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(
            Units.inchesToMeters(FrontLeftModule.LOCATION_FRONT_INCHES),
            Units.inchesToMeters(FrontLeftModule.LOCATION_LEFT_INCHES));
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(
            Units.inchesToMeters(FrontRightModule.LOCATION_FRONT_INCHES),
            Units.inchesToMeters(FrontRightModule.LOCATION_LEFT_INCHES));
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(
            Units.inchesToMeters(BackLeftModule.LOCATION_FRONT_INCHES),
            Units.inchesToMeters(BackLeftModule.LOCATION_LEFT_INCHES));
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(
            Units.inchesToMeters(BackRightModule.LOCATION_FRONT_INCHES),
            Units.inchesToMeters(BackRightModule.LOCATION_LEFT_INCHES));

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            FRONT_LEFT_LOCATION,
            FRONT_RIGHT_LOCATION,
            BACK_LEFT_LOCATION,
            BACK_RIGHT_LOCATION);

    // Pigeon2 (IMU/Gyro) (`swervedrive.json`)
    public static final class Pigeon {
        public static final int ID = 0;
    }

    // `modules/physicalproperties.json`
    public static final class PhysicalProperties {
        public static final int OPTIMAL_VOLTAGE_VOLTS = 12;

        // `currentLimit`
        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int ANGLE_CURRENT_LIMIT = 20;

        // `conversionFactors.angle`
        public static final double ANGLE_GEAR_RATIO = 21.4285714286;

        // `conversionFactors.drive`
        public static final double DRIVE_WHEEL_DIAMETER_INCHES = 4.0;
        public static final double DRIVE_GEAR_RATIO = 6.75;

        // `rampRate`
        public static final double DRIVE_RAMP_RATE_SECONDS = 0.1;
        public static final double ANGLE_RAMP_RATE_SECONDS = 0.1;
    }

    // `modules/pidfproperties.json`
    public static final class PIDFProperties {
        // `drive`
        public static final double DRIVE_kP = 0.020236;
        public static final double DRIVE_kI = 0.0;
        public static final double DRIVE_kD = 0.0;

        // `angle`
        public static final double ANGLE_kP = 0.016664;
        public static final double ANGLE_kI = 0.0;
        public static final double ANGLE_kD = 1.4884;
    }

    // `modules/backleft.json`
    public static final class BackLeftModule {
        public static final double LOCATION_FRONT_INCHES = -10.875;
        public static final double LOCATION_LEFT_INCHES = 10.875;

        public static final double ABSOLUTE_ENCODER_OFFSET_DEGREES = 148.380;

        public static final int DRIVE_ID = 8; // SparkMax Neo
        public static final int ANGLE_ID = 7; // SparkMax Neo
        public static final int ENCODER_ID = 4; // CanCoder

        public static final boolean DRIVE_INVERTED = true;
        public static final boolean ANGLE_INVERTED = true;
        public static final boolean ENCODER_INVERTED = false;
    }

    // `modules/backright.json`
    public static final class BackRightModule {
        public static final double LOCATION_FRONT_INCHES = -10.875;
        public static final double LOCATION_LEFT_INCHES = -10.875;

        public static final double ABSOLUTE_ENCODER_OFFSET_DEGREES = 271.270;

        public static final int DRIVE_ID = 6; // SparkMax Neo
        public static final int ANGLE_ID = 5; // SparkMax Neo
        public static final int ENCODER_ID = 3; // CanCoder

        public static final boolean DRIVE_INVERTED = true;
        public static final boolean ANGLE_INVERTED = true;
        public static final boolean ENCODER_INVERTED = false;
    }

    // `modules/frontleft.json`
    public static final class FrontLeftModule {
        public static final double LOCATION_FRONT_INCHES = 10.875;
        public static final double LOCATION_LEFT_INCHES = 10.875;

        public static final double ABSOLUTE_ENCODER_OFFSET_DEGREES = 250.64;

        public static final int DRIVE_ID = 2; // SparkMax Neo
        public static final int ANGLE_ID = 1; // SparkMax Neo
        public static final int ENCODER_ID = 1; // CanCoder

        public static final boolean DRIVE_INVERTED = true;
        public static final boolean ANGLE_INVERTED = true;
        public static final boolean ENCODER_INVERTED = false;
    }

    // `modules/frontright.json`
    public static final class FrontRightModule {
        public static final double LOCATION_FRONT_INCHES = 10.875;
        public static final double LOCATION_LEFT_INCHES = -10.875;

        public static final double ABSOLUTE_ENCODER_OFFSET_DEGREES = 296.333;

        public static final int DRIVE_ID = 4; // SparkMax Neo
        public static final int ANGLE_ID = 3; // SparkMax Neo
        public static final int ENCODER_ID = 2; // CanCoder

        public static final boolean DRIVE_INVERTED = true;
        public static final boolean ANGLE_INVERTED = true;
        public static final boolean ENCODER_INVERTED = false;
    }

    public static final class Simulation {
        public static final double kP = 0.05;
        public static final double kD = 0.0;
    }
}
