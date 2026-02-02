package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;

public class Constants {
    // TODO: Update values
    /** Location of hub on the field */
    public static final Translation2d HUB_LOCATION = new Translation2d(
            0,
            0);

    /** 20ms or 50hz */
    public static final Time LOOP_TIME = Seconds.of(0.02);
    /** Max speed of robot in meters per second */
    public static final double MAX_SPEED = Units.feetToMeters(10);
}
