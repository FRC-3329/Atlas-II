package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LEDConstants;

class RobotSimulationTest {
    private static RobotContainer robot;

    @BeforeAll
    static void initializeSimulation() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setJoystickAxisCount(0, 6);
        DriverStationSim.setJoystickButtonCount(0, 10);
        DriverStationSim.setJoystickPOVCount(0, 1);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        robot = new RobotContainer();
    }

    @AfterAll
    static void stopSimulation() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }

    @Test
    @Timeout(30)
    void simulatesRobotEndToEnd() {
        assertNotNull(robot.getSimulation());
        assertEquals(8, robot.getSimulation().getStoredFuelCount());
        assertEquals(0.25, robot.getIntake().getPivotAngle(), 0.01,
                "intake remote sensor should start at 90 degrees");

        CommandScheduler.getInstance().schedule(robot.getIntake().lower());
        runCycles(250);

        assertTrue(robot.getIntake().isDown());
        assertTrue(Math.abs(robot.getIntake().getPivotAngle())
                <= IntakeConstants.Pivot.PIVOT_TOLERANCE.in(Rotations),
                "intake pivot should reach its configured lower-setpoint tolerance; actual rotations="
                        + robot.getIntake().getPivotAngle());

        Pose2d initialPose = robot.getDrivebase().getSimulationPose();
        Command drive = robot.getDrivebase().driveFieldOriented(
                () -> new ChassisSpeeds(1.0, 0.0, 0.0));
        Command shoot = robot.getFlywheel().shoot(RPM.of(1600.0), Degrees.of(20.0));
        Command aim = robot.getTurret().moveToAngle(Degrees.of(30.0));
        Command feed = robot.getIndexer().feed();
        Command collect = robot.getIntake().intakeForward();

        CommandScheduler.getInstance().schedule(drive, shoot, aim, feed, collect);
        runCycles(250);

        Pose2d finalPose = robot.getDrivebase().getSimulationPose();
        assertTrue(finalPose.getTranslation().getDistance(initialPose.getTranslation()) > 1.0,
                "swerve ground-truth pose should move");
        assertTrue(Math.abs(robot.getFlywheel().getVelocityMeasure().in(RPM)) > 1400.0,
                "flywheel should spin to its closed-loop target; actual RPM="
                        + robot.getFlywheel().getVelocityMeasure().in(RPM));
        assertTrue(Math.abs(robot.getFlywheel().getRightVelocityMeasure().in(RPM)) > 1400.0,
                "right flywheel should spin to its closed-loop target; actual RPM="
                        + robot.getFlywheel().getRightVelocityMeasure().in(RPM));
        assertTrue(Math.abs(robot.getFlywheel().getHoodAngleMeasure().in(Degrees) - 20.0) < 3.0,
                "hood should reach its closed-loop target; actual degrees="
                        + robot.getFlywheel().getHoodAngleMeasure().in(Degrees));
        assertTrue(Math.abs(robot.getTurret().getAngleMeasure().in(Degrees) - 30.0) < 3.0,
                "turret motor and analog absolute encoder should agree at the target; actual degrees="
                        + robot.getTurret().getAngleMeasure().in(Degrees));
        assertTrue(Math.abs(robot.getIndexer().getIndexerVelocityRPM()) > 100.0);
        assertTrue(Math.abs(robot.getIndexer().getBeltVelocityRPM()) > 100.0);
        assertTrue(Math.abs(robot.getIntake().getRollerVelocityRPM()) > 100.0);

        assertTrue(robot.getFlywheel().getSimulationCurrentDrawAmps() >= 0.0);
        assertTrue(robot.getIndexer().getSimulationCurrentDrawAmps() > 0.0);
        assertTrue(robot.getIntake().getSimulationCurrentDrawAmps() > 0.0);
        assertTrue(robot.getTurret().getSimulationCurrentDrawAmps() >= 0.0);
        assertTrue(RobotController.getBatteryVoltage() < 13.5,
                "mechanism and drivetrain current should load the simulated battery");
        assertEquals(RobotController.getBatteryVoltage(), robot.getPdh().getVoltage(), 0.01,
                "the simulated PDH should report the loaded battery voltage");

        AddressableLEDSim ledSimulation = AddressableLEDSim.createForChannel(LEDConstants.PWM_PORT);
        assertTrue(ledSimulation.getInitialized());
        assertTrue(ledSimulation.getRunning());
        assertEquals(LEDConstants.STRIP_LENGTH, ledSimulation.getLength());

        assertTrue(robot.getSimulation().getLaunchedFuelCount() > 0,
                "feeding at flywheel speed should launch stored fuel into MapleSim");
        assertTrue(robot.getSimulation().getStoredFuelCount() < 8);
        assertTrue(robot.getAcceptedVisionEstimateCount() > 0,
                "simulated cameras should produce estimates consumed by odometry");

        robot.getSimulation().addFuel();
        CommandScheduler.getInstance().schedule(
                robot.getIndexer().feedBackwards(),
                robot.getIntake().intakeBackward());
        runCycles(20);
        assertTrue(robot.getSimulation().getEjectedFuelCount() > 0,
                "reversing the intake and indexer should return stored fuel to the field");
    }

    private static void runCycles(int cycles) {
        for (int i = 0; i < cycles; i++) {
            CommandScheduler.getInstance().run();
            robot.simulationPeriodic();
            SimHooks.stepTiming(0.02);
        }
    }
}
