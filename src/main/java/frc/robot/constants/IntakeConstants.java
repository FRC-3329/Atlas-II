package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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

        public static final double CRUISE_VELOCITY = 2.0;
        public static final double ACCELERATION = 1.0;
        public static final double JERK = 0.0;

        /** Parallel to ground — game pieces can roll in. */
        public static final Angle DOWN_ANGLE = Degrees.of(0.0);
        /** Perpendicular to ground — stowed position. */
        public static final Angle UP_ANGLE = Degrees.of(90.0);

        /** Angle at power-on before CANcoder sync; must match mechanical home. */
        public static final Angle STARTING_ANGLE = Degrees.of(147.4);

        public static final double CURRENT_LIMIT = 40.0;
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        /** Partially raised to flick fuel upward into the indexer. */
        public static final Angle KICK_ANGLE = Degrees.of(30.0);

        public static final int CANCODER_ID = 5;
        public static final Angle CANCODER_OFFSET = Degrees.of(-67.64);
        public static final SensorDirectionValue CANCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
        /** Total reduction from motor to pivot: 4:1 * 5:1 * 43:24 */
        public static final double GEARING_RATIO = 4.0 * 5.0 * 43.0 / 24.0;
    }

    public static final class Roller {
        public static final int MOTOR_ID = 12;

        public static final double INTAKE_VOLTAGE = 8.0;
        public static final double OUTTAKE_VOLTAGE = -5.0;

        public static final double VOLTAGE_COMPENSATION = 12.0;

        public static final int CURRENT_LIMIT = 80;
        public static final boolean INVERTED = false;
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    }
}
