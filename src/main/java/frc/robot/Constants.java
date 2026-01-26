package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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

        /**
         * How much to trust Quest vision measurements (lower = more trust) for pose
         * estimation
         */
        public static final Matrix<N3, N1> STD_Devs = VecBuilder.fill(
            0.02, // X position standard deviation in meters
            0.02, // Y position standard deviation in meters
            0.035 // Heading standard deviation in radians
        );
    }

    public static final class PVConstants {
        public static final String CAMERA_NAME = "Yellow Camera";
    
        /** The layout of the AprilTags on the field */
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeWelded);

        /** Transform from robot center to camera */
        public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
            Inches.of(5), // x, positive forward
            Inches.of(12.4), // y, positive left
            Inches.of(21.75), // z, positive up
            new Rotation3d(
                Degrees.of(0), // roll, counterclockwise rotation angle around the X axis
                Degrees.of(0), // pitch, counterclockwise rotation angle around the y axis
                Degrees.of(0) // yaw, counterclockwise rotation angle around the z axis
            )
        );

        /** time for the reading to be valid for */
        public static final Time VALID_TIME = Seconds.of(1.0);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // TODO: Fill in the correct values (these are placeholders)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    /** 20ms or 50hz */
    public static final Time LOOP_TIME = Seconds.of(0.02);
    /** Max speed of robot in meters per second */
    public static final double maxSpeed = Units.feetToMeters(10);
}