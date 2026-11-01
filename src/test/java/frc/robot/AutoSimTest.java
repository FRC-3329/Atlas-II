package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Runs one autonomous routine headless in simulation and checks that every
 * BLine path finishes without the bumpers touching the field. Run it with
 * {@code ./gradlew simAuto -Pauto="Left Fire First"}; see build.gradle for
 * the other options.
 */
class AutoSimTest {
    private static final double LOOP_SECONDS = 0.02;
    private static final double BUMPER_HALF_WIDTH = 0.889 / 2;
    private static final double FIELD_LENGTH = 16.541;
    private static final double FIELD_WIDTH = 8.052;
    // Trench walls as modeled by the YAGSL/maple-sim 2026 arena: centers, then size
    private static final double[][] TRENCH_WALLS = {
            {4.625, 1.432}, {4.625, 6.638}, {11.915, 1.432}, {11.915, 6.638}};
    private static final double TRENCH_WALL_LENGTH = 1.346;
    private static final double TRENCH_WALL_WIDTH = 0.305;

    @Test
    void runAuto() throws Exception {
        String auto = System.getProperty("auto", "Left Fire First");
        boolean red = Boolean.parseBoolean(System.getProperty("red", "false"));
        double seconds = Double.parseDouble(System.getProperty("seconds", "20"));

        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
        DriverStationSim.setAllianceStationId(red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        Robot robot = new Robot();
        robot.robotInit();
        Method loopFunc = IterativeRobotBase.class.getDeclaredMethod("loopFunc");
        loopFunc.setAccessible(true);
        SwerveSubsystem drivebase = (SwerveSubsystem) field(field(robot, Robot.class, "m_robotContainer"),
                RobotContainer.class, "drivebase");

        // Replaces the DogLog consumers from robotInit; this run only needs the values
        Map<String, Double> pathValues = new ConcurrentHashMap<>();
        int[] pathsStarted = {0};
        int[] pathsFinished = {0};
        double[] maxCrossTrack = {0};
        double[] lastPathEnd = {Double.NEGATIVE_INFINITY};
        FollowPath.setDoubleLoggingConsumer(entry -> {
            pathValues.put(entry.getFirst(), entry.getSecond());
            if (entry.getFirst().equals("FollowPath/crossTrackError")) {
                maxCrossTrack[0] = Math.max(maxCrossTrack[0], Math.abs(entry.getSecond()));
            }
        });
        FollowPath.setTranslationListLoggingConsumer(entry -> {
            if (entry.getFirst().equals("FollowPath/pathTranslations")) {
                pathsStarted[0]++;
                maxCrossTrack[0] = 0;
                System.out.printf("[%6.2f] path %d started at %s%n", Timer.getFPGATimestamp(), pathsStarted[0],
                        format(drivebase.getPose()));
            }
        });
        FollowPath.setBooleanLoggingConsumer(entry -> {
            if (entry.getFirst().equals("FollowPath/finished") && entry.getSecond()) {
                pathsFinished[0]++;
                lastPathEnd[0] = Timer.getFPGATimestamp();
                System.out.printf("[%6.2f] path %d finished at %s, max cross-track error %.3f m, %.0f events fired%n",
                        Timer.getFPGATimestamp(), pathsFinished[0], format(drivebase.getPose()), maxCrossTrack[0],
                        pathValues.getOrDefault("FollowPath/eventTriggersFiredCount", 0.0));
            }
        });
        CommandScheduler.getInstance().onCommandInitialize(command -> {
            // Anything starting mid-path came from an event trigger
            boolean duringPath = pathsStarted[0] > pathsFinished[0]
                    || Timer.getFPGATimestamp() - lastPathEnd[0] < 0.01;
            if (duringPath && !command.getName().equals("FollowPath")) {
                System.out.printf("[%6.2f]   event started %s at %s%n", Timer.getFPGATimestamp(), command.getName(),
                        format(drivebase.getPose()));
            }
        });
        CommandScheduler.getInstance().onCommandInterrupt(command -> {
            if (command.getName().equals("FollowPath")) {
                System.out.printf("[%6.2f] path interrupted at %s%n", Timer.getFPGATimestamp(),
                        format(drivebase.getPose()));
            }
        });

        // Let the dashboard and DS data settle, then pick the auto
        step(robot, loopFunc, 25);
        NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Auto Chooser")
                .getEntry("selected").setString(auto);
        applyGains(System.getProperty("gains"));
        step(robot, loopFunc, 5);

        String startPose = System.getProperty("startPose");
        if (startPose != null) {
            String[] parts = startPose.split(",");
            drivebase.resetOdometry(new Pose2d(Double.parseDouble(parts[0]), Double.parseDouble(parts[1]),
                    Rotation2d.fromDegrees(Double.parseDouble(parts[2]))));
        }
        System.out.printf("Running \"%s\" on %s from %s%n", auto, red ? "red" : "blue", format(drivebase.getPose()));

        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        double minClearance = Double.MAX_VALUE;
        Pose2d minClearancePose = drivebase.getPose();
        int loops = (int) Math.round(seconds / LOOP_SECONDS);
        for (int i = 0; i < loops; i++) {
            step(robot, loopFunc, 1);
            double clearance = clearance(drivebase.getPose());
            if (clearance < minClearance) {
                minClearance = clearance;
                minClearancePose = drivebase.getPose();
            }
            if (i % 25 == 0) {
                System.out.printf("  t=%5.2f %s remaining=%.2f m cross-track=%.3f m%n", i * LOOP_SECONDS,
                        format(drivebase.getPose()),
                        pathValues.getOrDefault("FollowPath/remainingPathDistanceMeters", 0.0),
                        pathValues.getOrDefault("FollowPath/crossTrackError", 0.0));
            }
        }

        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        System.out.printf("Ended at %s; %d/%d paths finished; closest to a wall: %.3f m at %s%n",
                format(drivebase.getPose()), pathsFinished[0], pathsStarted[0], minClearance,
                format(minClearancePose));
        assertEquals(pathsStarted[0], pathsFinished[0], "Every path that started should finish");
        assertTrue(minClearance >= 0, "Bumpers hit a wall at " + format(minClearancePose));
    }

    private static void step(Robot robot, Method loopFunc, int loops) throws Exception {
        for (int i = 0; i < loops; i++) {
            loopFunc.invoke(robot);
            // Async so the notifier TimedRobot never starts doesn't block the step
            SimHooks.stepTimingAsync(LOOP_SECONDS);
        }
    }

    /** Sets path-following gains through the same DogLog tunables the dashboard uses. */
    private static void applyGains(String gains) throws InterruptedException {
        if (gains == null) {
            return;
        }
        for (String gain : gains.split(",")) {
            String[] keyValue = gain.split("=");
            NetworkTableInstance.getDefault().getEntry("/Tunable/BLine/" + keyValue[0].trim())
                    .setDouble(Double.parseDouble(keyValue[1].trim()));
        }
        // DogLog polls tunables on its own thread
        Thread.sleep(500);
    }

    /** Smallest gap between the bumpers and a trench wall or the field border; negative means overlap. */
    private static double clearance(Pose2d pose) {
        double min = Double.MAX_VALUE;
        double cos = pose.getRotation().getCos();
        double sin = pose.getRotation().getSin();
        int samplesPerSide = 24;
        for (int k = 0; k < 4 * samplesPerSide; k++) {
            int side = k / samplesPerSide;
            double along = -BUMPER_HALF_WIDTH + 2 * BUMPER_HALF_WIDTH * (k % samplesPerSide) / samplesPerSide;
            double localX = side == 0 ? BUMPER_HALF_WIDTH : side == 1 ? -BUMPER_HALF_WIDTH : along;
            double localY = side == 2 ? BUMPER_HALF_WIDTH : side == 3 ? -BUMPER_HALF_WIDTH : along;
            double x = pose.getX() + localX * cos - localY * sin;
            double y = pose.getY() + localX * sin + localY * cos;
            min = Math.min(min, Math.min(Math.min(x, FIELD_LENGTH - x), Math.min(y, FIELD_WIDTH - y)));
            for (double[] wall : TRENCH_WALLS) {
                min = Math.min(min, boxDistance(x - wall[0], y - wall[1],
                        TRENCH_WALL_LENGTH / 2, TRENCH_WALL_WIDTH / 2));
            }
        }
        // Wall corners can poke into a bumper side between samples
        for (double[] wall : TRENCH_WALLS) {
            for (int sx = -1; sx <= 1; sx += 2) {
                for (int sy = -1; sy <= 1; sy += 2) {
                    double dx = wall[0] + sx * TRENCH_WALL_LENGTH / 2 - pose.getX();
                    double dy = wall[1] + sy * TRENCH_WALL_WIDTH / 2 - pose.getY();
                    min = Math.min(min, boxDistance(dx * cos + dy * sin, -dx * sin + dy * cos,
                            BUMPER_HALF_WIDTH, BUMPER_HALF_WIDTH));
                }
            }
        }
        return min;
    }

    /** Distance from a point to an axis-aligned box centered at the origin; negative inside. */
    private static double boxDistance(double dx, double dy, double halfX, double halfY) {
        double outsideX = Math.abs(dx) - halfX;
        double outsideY = Math.abs(dy) - halfY;
        if (outsideX > 0 || outsideY > 0) {
            return Math.hypot(Math.max(outsideX, 0), Math.max(outsideY, 0));
        }
        return Math.max(outsideX, outsideY);
    }

    private static Object field(Object target, Class<?> type, String name) throws ReflectiveOperationException {
        Field field = type.getDeclaredField(name);
        field.setAccessible(true);
        return field.get(target);
    }

    private static String format(Pose2d pose) {
        return String.format("(%.2f, %.2f, %.0f°)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }
}
