package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;

// TODO: Update values
public final class PVConstants {
        public static final String CAMERA_NAME = "Yellow Camera";

        /** The layout of the AprilTags on the field */
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltWelded);

        /** Transform from robot center to camera */
        public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
                Inches.of(0.0), // x, positive forward
                Inches.of(0.0), // y, positive left
                Inches.of(0.0), // z, positive up
                new Rotation3d(
                        Degrees.of(0), // roll, counterclockwise rotation angle around the X axis
                        Degrees.of(0), // pitch, counterclockwise rotation angle around the y axis
                        Degrees.of(0) // yaw, counterclockwise rotation angle around the z axis
                ));

        /** time for the reading to be valid for */
        public static final Time VALID_TIME = Seconds.of(1.0);

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0, 0, 0);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.0, 0.0, 0.0);
    }
