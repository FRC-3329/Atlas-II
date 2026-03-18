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
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Distance CAMERA_HEIGHT_Z = Inches.of(12.96);
    public static final Angle CAMERA_PITCH_UP = Degrees.of(25.0);

    public static final Transform3d A_ROBOT_TO_CAMERA = new Transform3d(
            Inches.of(-10.71), // x pos forward
            Inches.of(-8.15), // y pos left
            CAMERA_HEIGHT_Z, // z pos up
            new Rotation3d(
                    Degrees.zero(), // CCW rotation angle around the X axis (roll)
                    CAMERA_PITCH_UP.unaryMinus(), // CCW rotation angle around the Y axis (pitch)
                    Degrees.of(-108.1) // CCW rotation angle around the Z axis (yaw)
            ));

    public static final Transform3d B_ROBOT_TO_CAMERA = new Transform3d(
            Inches.of(-10.14), // x pos forward
            Inches.of(9.87), // y pos left
            CAMERA_HEIGHT_Z, // z pos up
            new Rotation3d(
                    Degrees.zero(), // CCW rotation angle around the X axis (roll)
                    CAMERA_PITCH_UP.unaryMinus(), // CCW rotation angle around the Y axis (pitch)
                    Degrees.of(180.0)// CCW rotation angle around the Z axis (yaw)
            ));

    public static final Transform3d C_ROBOT_TO_CAMERA = new Transform3d(
            Inches.of(-10.71), // x pos forward
            Inches.of(8.15), // y pos left
            CAMERA_HEIGHT_Z, // z pos up
            new Rotation3d(
                    Degrees.zero(), // CCW rotation angle around the X axis (roll)
                    CAMERA_PITCH_UP.unaryMinus(), // CCW rotation angle around the Y axis (pitch)
                    Degrees.of(108.1) // CCW rotation angle around the Z axis (yaw)
            ));

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 4);
}