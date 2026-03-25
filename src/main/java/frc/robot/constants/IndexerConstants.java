package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class IndexerConstants {
    public static final double FEED_VOLTAGE = 6.0;

    public static final double CURRENT_STALL_TIME = 0.5; // seconds
    public static final double SMART_REVERSAL_TIME = 0.2; // seconds

    public static final class Indexer {
        public static final int MOTOR_ID = 10;

        public static final int CURRENT_LIMIT = 60; // Amps
        public static final boolean INVERTED = true;
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    }

    public static final class Belt {
        public static final int MOTOR_ID = 11;

        public static final int CURRENT_LIMIT = 40; // Amps
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;
        public static final boolean INVERTED = false;
    }
}
