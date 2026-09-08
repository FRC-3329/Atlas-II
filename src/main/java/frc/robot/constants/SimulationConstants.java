package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

/** Robot-wide constants used only by desktop simulation. */
public final class SimulationConstants {
    private SimulationConstants() {
    }

    public static final class Fuel {
        private Fuel() {
        }

        /** The intake spans most of the front bumper and extends when lowered. */
        public static final Distance INTAKE_WIDTH = Inches.of(26.0);
        public static final Distance INTAKE_EXTENSION = Inches.of(10.0);
        public static final int CAPACITY = 40;
        public static final int PRELOAD_COUNT = 8;

        public static final Distance SHOOTER_HEIGHT = Inches.of(24.0);
        public static final double LAUNCH_EFFICIENCY = 0.55;
        public static final double SHOT_INTERVAL_SECONDS = 0.12;
        public static final double MIN_SHOOTING_RPM = 500.0;
    }
}
