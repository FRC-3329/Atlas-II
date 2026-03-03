package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class PVConstants {
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Distance CAMERA_HEIGHT_Z = Inches.of(7.675);
    public static final Angle CAMERA_PITCH_UP = Degrees.of(15.0);

    public static final Transform3d BLUE_ROBOT_TO_CAMERA = new Transform3d(
        Inches.of(-7.819), // x pos forward
        Inches.of(-11.294), // x pos left 
        CAMERA_HEIGHT_Z, // z pos up
        new Rotation3d(
            Degrees.zero(), // CCW rotation angle around the X axis (roll)
            CAMERA_PITCH_UP.unaryMinus(), // CCW rotation angle around the Y axis (pitch)
            Degrees.of(276.6 - 30.9) // CCW rotation angle around the Z axis (yaw)
        )
    );

    public static final Transform3d ORANGE_ROBOT_TO_CAMERA = new Transform3d(
        Inches.of(-11.250), // x pos forward
        Inches.of(-8.573), // x pos left 
        CAMERA_HEIGHT_Z, // z pos up
        new Rotation3d(
            Degrees.zero(), // CCW rotation angle around the X axis (roll)
            CAMERA_PITCH_UP.unaryMinus(), // CCW rotation angle around the Y axis (pitch)
            Degrees.of(203.7 - 30.9)// CCW rotation angle around the Z axis (yaw)
        )
    );

    public static final Transform3d YELLOW_ROBOT_TO_CAMERA = new Transform3d(
        Inches.of(-11.294), // x pos forward
        Inches.of(7.819), // x pos left 
        CAMERA_HEIGHT_Z, // z pos up
        new Rotation3d(
            Degrees.zero(), // CCW rotation angle around the X axis (roll)
            CAMERA_PITCH_UP.unaryMinus(), // CCW rotation angle around the Y axis (pitch)
            Degrees.of(156.3 + 30.9) // CCW rotation angle around the Z axis (yaw)
        )
    );

    public static final Transform3d RED_ROBOT_TO_CAMERA = new Transform3d(
        Inches.of(-8.573), // x pos forward
        Inches.of(11.250), // x pos left 
        CAMERA_HEIGHT_Z, // z pos up
        new Rotation3d(
            Degrees.zero(), // CCW rotation angle around the X axis (roll)
            CAMERA_PITCH_UP.unaryMinus(), // CCW rotation angle around the Y axis (pitch)
            Degrees.of(83.4 + 30.9) // CCW rotation angle around the Z axis (yaw)
        )
    );

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 4); 
}