package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;

public final class TurretConstants {
    public static final int MOTOR_ID = 4;

    public static final double kP = 79.6;
    public static final double kI = 0.0;
    public static final double kD = 3.8253;
    public static final double kS = 0.10193;
    public static final double kV = 3.5;
    public static final double kA = 0.12559;

    public static final double CRUISE_VELOCITY = 10.0; // rotations per second
    public static final double ACCELERATION = 3.0; // rotations per second squared
    public static final double JERK = 0.0; // rotations per second cubed

    public static final Angle MIN_ANGLE = Degrees.of(-60.0);
    public static final Angle MAX_ANGLE = Degrees.of(60.0);

    public static final double CURRENT_LIMIT = 30.0; // Amps
    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final double MANUAL_SPEED = 0.05; // Percent output for manual movement

    public static final int ABSOLUTE_ENCODER_CHANNEL = 0; // 0-3 on-board, 4-7 on MXP
    public static final double ABSOLUTE_ENCODER_FULL_RANGE = 360.0; // Degrees for full rotation
    public static final double ABSOLUTE_ENCODER_OFFSET = -180.0; // Offset in degrees
    public static final boolean USE_ABSOLUTE_ENCODER = true;

    public static final double TOP_FREE_SPACE_Y_OFFSET = 1.5; // meters
    public static final double BOTTOM_FREE_SPACE_Y_OFFSET = 1.5; // meters

    public static final boolean ENABLE_AUTO_AIM = false;

    public static final double LED_TOLERANCE_DEGREES = 2.0;

    public static final Transform2d ROBOT_TO_TURRET = new Transform2d(Inches.of(-6.500), Inches.zero(),
            Rotation2d.kZero);
}