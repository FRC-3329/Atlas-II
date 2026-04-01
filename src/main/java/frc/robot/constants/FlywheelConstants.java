package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public final class FlywheelConstants {
    /** Fallback values used when distance-based (varying) RPM is disabled. */
    public static final AngularVelocity STATIC_RPM = RPM.of(3000.0);
    public static final Angle STATIC_HOOD_ANGLE = Degrees.of(45.0);

    public static final class Flywheel {
        public static final int LEFT_ID = 2;
        public static final int RIGHT_ID = 1;

        // SysId-characterized feedforward + PID gains
        public static final double kP = 0.7;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.11085;
        public static final double kV = 0.11419;
        public static final double kA = 0.023967;

        public static final double CRUISE_VELOCITY = 10000.0;
        public static final double ACCELERATION = 10000.0;

        public static final double CURRENT_LIMIT = 60.0;
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
    }

    public static final class Hood {
        public static final int HOOD_ID = 6;

        // SysId-characterized feedforward + PID gains (arm w/ gravity comp)
        public static final double kP = 1000.0;
        public static final double kI = 0.0;
        public static final double kD = 0.05;
        public static final double kS = 0.0;
        public static final double kV = 2.2;
        public static final double kA = 0.01;
        public static final double kG = 0.65;

        public static final double GRAVITY_ARM_POSITION_OFFSET = 0.0;

        public static final double CRUISE_VELOCITY = 10.0;
        public static final double ACCELERATION = 2.0;

        public static final double CURRENT_LIMIT = 25.0;
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        /** Slow reverse voltage used during zeroing to detect hard-stop via current spike. */
        public static final double ZEROING_VOLTAGE = -0.5;
        public static final double ZEROING_CURRENT_THRESHOLD = 5.0;
        public static final double STALL_DEBOUNCE_TIME = 0.1;
    }
}