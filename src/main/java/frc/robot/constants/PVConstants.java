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

    /** Shared across all cameras — they're mounted at the same height. */
    public static final Distance CAMERA_HEIGHT_Z = Inches.of(7.69);
    /** Tilted up to see tags at range while the robot frame stays low. */
    public static final Angle CAMERA_PITCH_UP = Degrees.of(25.0);

    /** Left Side. */
    public static final Transform3d A_ROBOT_TO_CAMERA = new Transform3d(
            Inches.of(-7.81),
            Inches.of(-11.14),
            CAMERA_HEIGHT_Z,
            new Rotation3d(
                    Degrees.zero(),
                    CAMERA_PITCH_UP.unaryMinus(),
                    Degrees.of(20.0)));

    /** Left Front. */
    public static final Transform3d B_ROBOT_TO_CAMERA = new Transform3d(
            Inches.of(-11.08),
            Inches.of(-8.47),
            Inches.of(7.78),
            new Rotation3d(
                    Degrees.zero(),
                    CAMERA_PITCH_UP.unaryMinus(),
                    Degrees.of(35.0)));

    /** Right Side. */
    public static final Transform3d C_ROBOT_TO_CAMERA = new Transform3d(
            Inches.of(-7.81),
            Inches.of(11.14),
            CAMERA_HEIGHT_Z,
            new Rotation3d(
                    Degrees.zero(),
                    CAMERA_PITCH_UP.unaryMinus(),
                    Degrees.of(20.0)));

    /** Right Front. */
    public static final Transform3d D_ROBOT_TO_CAMERA = new Transform3d(
            Inches.of(-11.11),
            Inches.of(8.47),
            CAMERA_HEIGHT_Z,
            new Rotation3d(
                    Degrees.zero(),
                    CAMERA_PITCH_UP.unaryMinus(),
                    Degrees.of(20.0)));

    /** Higher std devs = less trust; single tags have more ambiguity. */
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    /** Multi-tag solves are much more reliable, so we trust them more. */
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 4);
}