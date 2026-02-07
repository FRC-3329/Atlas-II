package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

// TODO: Update values
public class OrientToHubConstants {
    public static final double ORIENT_TO_HUB_KP = 0.0;
    public static final double ORIENT_TO_HUB_KI = 0.0;
    public static final double ORIENT_TO_HUB_KD = 0.0;

    public static final AngularVelocity ORIENT_TO_HUB_MAX_VELOCITY = DegreesPerSecond.of(0.0);
    public static final AngularAcceleration ORIENT_TO_HUB_MAX_ACCELERATION = DegreesPerSecond.per(Seconds).of(0.0);
    public static final Angle ORIENT_TO_HUB_TOLERANCE = Degrees.of(0.0);
}