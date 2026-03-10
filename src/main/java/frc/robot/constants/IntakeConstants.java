package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;

public final class IntakeConstants {
    public static final class Pivot {
        public static final int MOTOR_ID = 7;

        public static final double kP = 34.46;
        public static final double kI = 0.0;
        public static final double kD = 0.52;
        public static final double kS = 0.0;
        public static final double kV = 4.25;
        public static final double kA = 0.00;
        public static final double kG = 0.24;
        
        public static final double GRAVITY_ARM_POSITION_OFFSET = 0.0; // rotations

        public static final double CRUISE_VELOCITY = 2.0; // rotations per second
        public static final double ACCELERATION = 1.0; // rotations per second squared
        public static final double JERK = 0.0; // rotations per second cubed

        /** Down position - parallel to ground */
        public static final Angle DOWN_ANGLE = Degrees.of(-25.0);
        /** Up position - perpendicular to ground */
        public static final Angle UP_ANGLE = Degrees.of(90.0);

        public static final int ABSOLUTE_ENCODER_PORT = 9;
        /** Offset to zero the absolute encoder */
        public static final Angle ABSOLUTE_ENCODER_OFFSET = Rotations.of(11.43);

        public static final Angle STARTING_ANGLE = Degrees.of(147.4);

        public static final double CURRENT_LIMIT = 40.0; // Amps
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final Angle KICK_ANGLE = Degrees.of(40.0);
    }

    public static final class Roller {
        public static final int MOTOR_ID = 11;

        public static final double INTAKE_VOLTAGE = 11.0; // 75% output
        public static final double OUTTAKE_VOLTAGE = -9.5;

        public static final double VOLTAGE_COMPENSATION = 12.0;

        public static final int CURRENT_LIMIT = 40; // Amps
        public static final boolean INVERTED = true;
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    }
}
