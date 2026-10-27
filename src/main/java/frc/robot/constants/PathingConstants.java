package frc.robot.constants;

/**
 * Gains for the BLine path follower. Kinematic limits and handoff radii live in
 * deploy/autos/config.json so the BLine web editor can read and write them.
 */
public final class PathingConstants {
    private PathingConstants() {
    }

    /** Drives the speed command by minimizing the remaining path distance. */
    // TODO
    public static final double TRANSLATION_kP = 0.0;
    // TODO
    public static final double TRANSLATION_kI = 0.0;
    // TODO
    public static final double TRANSLATION_kD = 0.0;

    /** Turns the robot toward the active rotation target. */
    // TODO
    public static final double ROTATION_kP = 0.0;
    // TODO
    public static final double ROTATION_kI = 0.0;
    // TODO
    public static final double ROTATION_kD = 0.0;

    /** Pulls the robot back onto the line between translation targets. */
    // TODO
    public static final double CROSS_TRACK_kP = 0.0;
    // TODO
    public static final double CROSS_TRACK_kI = 0.0;
    // TODO
    public static final double CROSS_TRACK_kD = 0.0;
}
