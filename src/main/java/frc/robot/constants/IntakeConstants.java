package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;

// TODO: Update values
public final class IntakeConstants {
    public static final class Pivot {
        public static final int MOTOR_ID = 0;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kG = 0.0;
        
        // TODO: Check with CAD
        public static final double GRAVITY_ARM_POSITION_OFFSET = 0.0;

        public static final double CRUISE_VELOCITY = 0.0; // rotations per second
        public static final double ACCELERATION = 0.0; // rotations per second squared
        public static final double JERK = 0.0; // rotations per second cubed

        /** Down position - parallel to ground */
        public static final Angle DOWN_ANGLE = Degrees.of(0.0);
        /** Up position - perpendicular to ground */
        public static final Angle UP_ANGLE = Degrees.of(90.0);

        public static final int ABSOLUTE_ENCODER_PORT = 0;
        /** Offset to zero the absolute encoder */
        public static final Angle ABSOLUTE_ENCODER_OFFSET = Rotations.of(0.0);

        public static final double CURRENT_LIMIT = 40.0; // Amps
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
    }

    public static final class Roller {
        public static final int MOTOR_ID = 0;

        public static final double INTAKE_VOLTAGE = 4.0; // ~33% output
        public static final double OUTTAKE_VOLTAGE = -4.0;

        public static final double VOLTAGE_COMPENSATION = 12.0;

        public static final int CURRENT_LIMIT = 30; // Amps
        public static final boolean INVERTED = false;
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    }
}
