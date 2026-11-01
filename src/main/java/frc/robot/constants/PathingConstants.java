package frc.robot.constants;

/**
 * Gains for the BLine path follower. Kinematic limits and handoff radii live in
 * deploy/autos/config.json so the BLine web editor can read and write them.
 * <p>
 * Every gain here is also a DogLog tunable under /Tunable/BLine, so it can be
 * tuned live from the dashboard. Copy the tuned values back here afterwards,
 * since dashboard changes are lost when the robot code restarts.
 */
public final class PathingConstants {
    private PathingConstants() {
    }

    /** Drives the speed command by minimizing the remaining path distance. */
    public static final double TRANSLATION_kP = 2.0;
    public static final double TRANSLATION_kI = 0.0;
    public static final double TRANSLATION_kD = 0.0;

    /**
     * Turns the robot toward the active rotation target. Too low and the robot
     * stalls a few degrees short of the final heading and the path never ends,
     * because YAGSL won't re-steer the modules for very small speed requests.
     */
    public static final double ROTATION_kP = 4.0;
    public static final double ROTATION_kI = 0.0;
    public static final double ROTATION_kD = 0.0;

    /**
     * Pulls the robot back onto the line between translation targets. Keep kI at
     * zero: BLine never resets this controller, so integral would carry between paths.
     */
    public static final double CROSS_TRACK_kP = 2.0;
    public static final double CROSS_TRACK_kI = 0.0;
    public static final double CROSS_TRACK_kD = 0.0;

    /** How far the tuning autos drive forward; also a DogLog tunable. */
    public static final double TUNING_PATH_DISTANCE_METERS = 3.0;
    /**
     * How far off the line the cross-track tuning auto starts. Must stay below the
     * handoff radius in config.json, or the robot drives sideways onto the line first.
     */
    public static final double TUNING_CROSS_TRACK_OFFSET_METERS = 0.25;
}
