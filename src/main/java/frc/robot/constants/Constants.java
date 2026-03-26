package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;

public class Constants {
    /** Blue-alliance-relative position of the hub center on the field (meters). */
    public static final Translation2d HUB_LOCATION = new Translation2d(
            4.624,
            4.035);

    /** TimedRobot loop period — 20ms matches the default CAN frame rate. */
    public static final Time LOOP_TIME = Seconds.of(0.02);
    public static final double MAX_SPEED = Units.feetToMeters(10);
    /** Shots closer than this hit the hub rim; shots farther lose accuracy. */
    public static final double MIN_SHOT_DISTANCE = 1.9;
    public static final double MAX_SHOT_DISTANCE = 4.02;
}
