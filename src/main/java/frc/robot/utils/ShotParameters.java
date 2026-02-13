package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;

// credit to 1683
// https://github.com/TechnoTitans/TitanWare2024/blob/master/src/main/java/frc/robot/subsystems/superstructure/ShootOnTheMove.java
public class ShotParameters {
    /** The parameters we characterize for each distance key in the map */
    public record Parameters(double flywheelRPS, double hoodRotations, double tofSeconds)
            implements Interpolatable<Parameters> {

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
        map.put(1.0, new Parameters(10, 10, 1));
        map.put(2.0, new Parameters(10, 10, 1.5));
    }

}