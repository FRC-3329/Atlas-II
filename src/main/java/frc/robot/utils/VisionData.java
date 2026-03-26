package frc.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionData {
    @FunctionalInterface
    public static interface EstimateConsumer {
        /**
         * Feeds a vision measurement into the drivetrain's pose estimator.
         * Lower std dev values = higher confidence in this measurement.
         */
        public void accept(
                Pose3d pose,
                double timestamp,
                Matrix<N3, N1> estimationStdDevs);
    }
}
