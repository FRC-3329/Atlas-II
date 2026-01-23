package frc.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Utility class for vision-based pose estimation data. */
public class VisionData {
    /**
     * Functional interface for consuming vision pose estimates.
     * 
     * This callback interface is used to feed vision measurements into a pose estimator (SwerveDrivePoseEstimator). 
     * Vision systems like QN or PV use this interface to provide their pose measurements to the drivetrain's pose estimation system.
     * 
     * The pose estimator uses a Kalman filter to fuse vision data with wheel odometry and gyro measurements, 
     * using the standard deviations to determine how much to trust each measurement source.
     */
    @FunctionalInterface
    public static interface EstimateConsumer {
        /**
         * Accepts a vision pose measurement and adds it to the pose estimator.
         * 
         * @param pose The measured robot pose in 3D space from the vision system
         * @param timestamp The timestamp when this measurement was taken (in seconds, from FPGA timer)
         * @param estimationStdDevs Standard deviations for the measurement (x, y, heading).
         *                          Lower values indicate higher confidence in the measurement.
         *                          These values control how much the pose estimator trusts this
         *                          vision measurement compared to wheel odometry.
         */
        public void accept(
            Pose3d pose,
            double timestamp,
            Matrix<N3, N1> estimationStdDevs
        );
    }
}
