package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public final class FlywheelConstants {
    // Static shooting parameters when not using positional information
    // TODO: Update values
    public static final AngularVelocity STATIC_RPM = RPM.of(3000.0);
    public static final Angle STATIC_HOOD_ANGLE = Degrees.of(45.0);

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

    public static final class Hood {
        public static final int HOOD_ID = 6;

        public static final double kP = 80.0; // V/rotation
        public static final double kI = 0.0;
        public static final double kD = 0.05; // V*s/rotation
        public static final double kS = 0.0; // V
        public static final double kV = 2.2; // V*s/rotation
        public static final double kA = 0.01; // v*s^2/rotation
        public static final double kG = 0.0; // V

        // TODO: Check with CAD
        public static final double GRAVITY_ARM_POSITION_OFFSET = 0.0; // rotations

        public static final double CRUISE_VELOCITY = 10.0;
        public static final double ACCELERATION = 2.0;
        // public static final double JERK = 0.0;

        public static final double CURRENT_LIMIT = 25.0; // Amps
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final double ZEROING_VOLTAGE = -2.0; // Volts
        public static final double ZEROING_CURRENT_THRESHOLD = 15.0; // Amps
    }
}