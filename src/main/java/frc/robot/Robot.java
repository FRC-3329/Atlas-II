package frc.robot;

import com.pathplanner.lib.util.PathPlannerLogging;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.TurretConstants;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void robotInit() {
        // Capture all network table + driver station data for post-match analysis
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
        PathPlannerLogging.setLogTargetPoseCallback(pose -> DogLog.log("PathPlanner/TargetPose", pose));
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        // Brake mode prevents drift between path segments
        m_robotContainer.setMotorBrake(true);

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // Always auto-aim at competition; constant gate allows practice without FMS
        if (DriverStation.isFMSAttached() || TurretConstants.ENABLE_AUTO_AIM) {
            CommandScheduler.getInstance().schedule(m_robotContainer.getTurretAutoTrack());
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
        m_robotContainer.simulationPeriodic();
    }
}
