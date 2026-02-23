package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class IndexerConstants {
    public static final int MOTOR_ID = 10;

    public static final double FEED_VOLTAGE = 3.0;

    public static final int CURRENT_LIMIT = 40; // Amps
    public static final boolean INVERTED = true;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
}
