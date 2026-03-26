package frc.robot.constants;

public final class TrenchConstants {
    private TrenchConstants() {
    }

    public static final double FIELD_LENGTH = 16.54;
    public static final double FIELD_WIDTH = 8.07;

    public static final String PATH_A = "Trench A";
    public static final String PATH_B = "Trench B";
    public static final String PATH_C = "Trench C";
    public static final String PATH_D = "Trench D";

    /** Slower than normal driving so we don't overshoot the trench entrance. */
    public static final double MAX_VELOCITY = 1.0;
    public static final double MAX_ACCELERATION = 0.5;
    public static final double MAX_ANGULAR_VELOCITY = 540.0;
    public static final double MAX_ANGULAR_ACCELERATION = 720.0;
}
