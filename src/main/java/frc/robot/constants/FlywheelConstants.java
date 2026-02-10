package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;

public final class FlywheelConstants {
    public static final class Flywheel {
        public static final int LEFT_ID = 2;
        public static final int RIGHT_ID = 1;

        public static final double kP = 0.10937;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.11085;
        public static final double kV = 0.11419;
        public static final double kA = 0.023967;

        public static final double CRUISE_VELOCITY = 10000.0;
        public static final double ACCELERATION = 10000.0;
        // public static final double JERK = 0.0;

        public static final double CURRENT_LIMIT = 45.0; // Amps
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
    }

    // TODO: Populate hood constants
    public static final class Hood {
        public static final int HOOD_ID = 0;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kG = 0.0;
        
        // TODO: Check with CAD
        public static final double GRAVITY_ARM_POSITION_OFFSET = 0.0; // rotations

        public static final double CRUISE_VELOCITY = 0.0;
        public static final double ACCELERATION = 0.0;
        public static final double JERK = 0.0;

        public static final double CURRENT_LIMIT = 25.0; // Amps
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
    }
}