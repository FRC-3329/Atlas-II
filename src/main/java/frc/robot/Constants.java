package frc.robot;

import edu.wpi.first.math.util.Units;

/** Constants */
public final class Constants {
    //* Constants used for operator controls */
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final double DEADBAND = 0.01;
    }

    /** Maximum speed of the robot in meters per second */
    public static final double maxSpeed = Units.feetToMeters(10);
}