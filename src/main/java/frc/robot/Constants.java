package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;

public final class Constants {
    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double DEADBAND = 0.01;
    }

    // TODO: Update values
    public static final class QNConstants {
        /** Physical offset from robot center to Quest headset mounting location */
        public static final Transform3d Robot_to_Quest = new Transform3d(
            Meters.of(-0.222), // X translation from robot center to Quest in meters
            Meters.of(0.162), // Y translation from robot center to Quest in meters
            Meters.of(0.0), // Z translation (height) from robot center to Quest in meters
            new Rotation3d(0, 0, Math.PI) // 180 degree rotation around Z axis
        );

        /** How much to trust Quest vision measurements (lower = more trust) for pose estimation */
        public static final Matrix<N3, N1> STD_Devs = VecBuilder.fill(
            0.02, // X position standard deviation in meters
            0.02, // Y position standard deviation in meters
            0.035 // Heading standard deviation in radians
        );
    }

    /** 20ms or 50hz */
    public static final Time LOOP_TIME = Seconds.of(0.02);
    /** Max speed of robot in meters per second */
    public static final double maxSpeed = Units.feetToMeters(10);
}