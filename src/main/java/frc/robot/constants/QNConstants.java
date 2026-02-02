package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

// TODO: Update values
public final class QNConstants {
        /** Physical offset from robot center to Quest headset mounting location */
        public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
                Meters.of(0.0), // X translation from robot center to Quest in meters
                Meters.of(0.0), // Y translation from robot center to Quest in meters
                Meters.of(0.0), // Z translation (height) from robot center to Quest in meters
                new Rotation3d(0, 0, Math.PI) // 180 degree rotation around Z axis
        );

        /**
         * How much to trust Quest vision measurements (lower = more trust) for pose
         * estimation
         */
        public static final Matrix<N3, N1> STD_DEVS = VecBuilder.fill(
                0.02, // X position standard deviation in meters
                0.02, // Y position standard deviation in meters
                0.035 // Heading standard deviation in radians
        );
}
