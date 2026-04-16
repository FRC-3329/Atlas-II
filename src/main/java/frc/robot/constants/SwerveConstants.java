package frc.robot.constants;

// Adapted from previous YAGSL config files
public class SwerveConstants {
    // Pigeon2 (IMU) (`swervedrive.json`)
    public static final class Pigeon {
        public static final int ID = 0;
        public static final boolean INVERTED = false;
    }

    // `controllerproperties.json`
    public static final class ControllerProperties {
        public static final double DEADBAND = 0.5;
    
        public static final double kP = 0.4;
        public static final double kI = 0.0;
        public static final double kD = 0.01;
    }

    // `modules/physicalproperties.json`
    public static final class PhysicalProperties {
        public static final int OPTIMAL_VOLTAGE = 12;
        public static final double ROBOT_MASS = 135.0;
        public static final double WHEEL_GRIP_COEFFICIENT_OF_FRICTION = 1.0;

        // `currentLimit`
        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int ANGLE_CURRENT_LIMIT = 20;

        // `conversionFactors.angle`
        public static final double ANGLE_GEAR_RATIO = 21.4285714286;
        public static final double ANGLE_FACTOR = 0.0;
    
        // `conversionFactors.drive`
        public static final double DRIVE_DIAMETER = 4.0;
        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double DRIVE_FACTOR = 0.0;

        // `rampRate`
        public static final double DRIVE_RAMP_RATE = 0.1;
        public static final double ANGLE_RAMP_RATE = 0.1;
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
        public static final double ANGLE_kF = 0.0;
        public static final double angle_kIZ = 0.0;
    }

    // `modules/backleft.json`
    public static final class BackLeftModule {
        public static final double LOCATION_FRONT = -10.875;
        public static final double LOCATION_LEFT = 10.875;

        public static final double ABSOLUTE_ENCODER_OFFSET = 148.380;

        public static final int DRIVE_ID = 8; // SparkMax Neo
        public static final int ANGLE_ID = 7; // SparkMax Neo
        public static final int ENCODER_ID = 4; // CanCoder

        public static final boolean DRIVE_INVERTED = true;
        public static final boolean ANGLE_INVERTED = true;
        public static final boolean ENCODER_INVERTED = false;
    }

    // `modules/backright.json`
    public static final class BackRightModule {
        public static final double LOCATION_FRONT = -10.875;
        public static final double LOCATION_LEFT = -10.875;

        public static final double ABSOLUTE_ENCODER_OFFSET = 271.270;

        public static final int DRIVE_ID = 6; // SparkMax Neo
        public static final int ANGLE_ID = 5; // SparkMax Neo
        public static final int ENCODER_ID = 3; // CanCoder

        public static final boolean DRIVE_INVERTED = true;
        public static final boolean ANGLE_INVERTED = true;
        public static final boolean ENCODER_INVERTED = false;
    }

    // `modules/frontleft.json`
    public static final class FrontLeftModule {
        public static final double LOCATION_FRONT = 10.875;
        public static final double LOCATION_LEFT = 10.875;

        public static final double ABSOLUTE_ENCODER_OFFSET = 250.64;

        public static final int DRIVE_ID = 2; // SparkMax Neo
        public static final int ANGLE_ID = 1; // SparkMax Neo
        public static final int ENCODER_ID = 1; // CanCoder

        public static final boolean DRIVE_INVERTED = true;
        public static final boolean ANGLE_INVERTED = true;
        public static final boolean ENCODER_INVERTED = false;
    }

    // `modules/frontright.json`
    public static final class FrontRightModule {
        public static final double LOCATION_FRONT = 10.875;
        public static final double LOCATION_LEFT = -10.875;

        public static final double ABSOLUTE_ENCODER_OFFSET = 296.333;

        public static final int DRIVE_ID = 4; // SparkMax Neo
        public static final int ANGLE_ID = 3; // SparkMax Neo
        public static final int ENCODER_ID = 2; // CanCoder

        public static final boolean DRIVE_INVERTED = true;
        public static final boolean ANGLE_INVERTED = true;
        public static final boolean ENCODER_INVERTED = false;
    }
}
