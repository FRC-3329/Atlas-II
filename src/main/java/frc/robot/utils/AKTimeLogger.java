package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotController;

public final class AKTimeLogger {
    private static final Map<String, Long> timingStarts = new HashMap<>();

    private AKTimeLogger() {}

    public static void startTiming(String key) {
        timingStarts.put(key, RobotController.getFPGATime());
    }

    public static void endTiming(String key) {
        Long start = timingStarts.remove(key);

        if (start != null) {
            Logger.recordOutput(key, (RobotController.getFPGATime() - start) / 1_000_000.0);
        }
    }
}
