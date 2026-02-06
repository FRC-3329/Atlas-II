package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

// TODO: Update values
public final class PVConstants {
        /** The layout of the AprilTags on the field */
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.k2026RebuiltWelded);

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0, 0, 0);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.0, 0.0, 0.0);
}