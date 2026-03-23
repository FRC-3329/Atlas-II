package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

// credit to 1683
// https://github.com/TechnoTitans/TitanWare2024/blob/master/src/main/java/frc/robot/subsystems/superstructure/ShootOnTheMove.java
public class ShotParameters {
    /** The parameters we characterize for each distance key in the map */
    public record Parameters(double flywheelRPS, double hoodRotations, double tofSeconds)
            implements Interpolatable<Parameters> {

        public Parameters(AngularVelocity flywheelAngularVelocity, Angle hoodAngle, Time tof) {
            this(flywheelAngularVelocity.in(RotationsPerSecond), hoodAngle.in(Rotations), tof.in(Seconds));
        }

        @Override
        public Parameters interpolate(Parameters endValue, double t) {
            return new Parameters(
                    MathUtil.interpolate(this.flywheelRPS, endValue.flywheelRPS, t),
                    MathUtil.interpolate(this.hoodRotations, endValue.hoodRotations, t),
                    MathUtil.interpolate(this.tofSeconds, endValue.tofSeconds, t));
        }
    };

    private static final InterpolatingTreeMap<Double, Parameters> map = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            Parameters::interpolate);

    /**
     * @param distanceMeters Distance from the target in meters
     * @return Shot parameters for the distance from the target
     */
    public static Parameters getShotParameters(double distanceMeters) {
        return map.get(distanceMeters);
    }

    /**
     * @param distanceMeters Distance from the target
     * @return Shot parameters for the distance from the target
     */
    public static Parameters getShotParameters(Distance distance) {
        return map.get(distance.in(Meters));
    }

    // add to the map
    static {
        // key units are in meters
        map.put(1.84, new Parameters(RPM.of(1450.0), Degrees.of(0.0), Seconds.of(0.89)));
        map.put(2.2, new Parameters(RPM.of(1550.0), Degrees.of(4.0), Seconds.of(1.01)));
        map.put(2.6, new Parameters(RPM.of(1650.0), Degrees.of(7.0), Seconds.of(1.15)));
        map.put(3.15, new Parameters(RPM.of(1650.0), Degrees.of(16.7), Seconds.of(1.075)));
        map.put(4.0, new Parameters(RPM.of(1710.0), Degrees.of(20.5), Seconds.of(1.11)));

    }

}