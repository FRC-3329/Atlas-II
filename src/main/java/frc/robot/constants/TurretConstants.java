package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;

// TODO: Update values
public final class TurretConstants {
    public static final int MOTOR_ID = 0;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final double CRUISE_VELOCITY = 0.0; // rotations per second
    public static final double ACCELERATION = 0.0; // rotations per second squared
    public static final double JERK = 0.0; // rotations per second cubed

    public static final double MIN_ANGLE = 0.0;
    public static final double MAX_ANGLE = 0.0;

    public static final double CURRENT_LIMIT = 30.0; // Amps
    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final double MANUAL_SPEED = 0.2; // Percent output for manual movement

    public static final int ABSOLUTE_ENCODER_CHANNEL = 0; // 0-3 on-board, 4-7 on MXP
    public static final double ABSOLUTE_ENCODER_FULL_RANGE = 360.0; // Degrees for full rotation
    public static final double ABSOLUTE_ENCODER_OFFSET = -180.0; // Offset in degrees
    public static final boolean USE_ABSOLUTE_ENCODER = false;

    /**
     * The angle offset from hub center to the top free space when aiming from
     * opponent zone
     */
    public static final double TOP_FREE_SPACE_ANGLE_OFFSET = 45.0; // degrees
    /**
     * The angle offset from hub center to the bottom free space when aiming from
     * opponent zone
     */
    public static final double BOTTOM_FREE_SPACE_ANGLE_OFFSET = -45.0; // degrees
}