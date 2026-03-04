package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.TrenchConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command to automatically drive the robot under the nearest trench.
 */
public class AutoDriveUnderTrenchCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;
    private final PathConstraints pathfindingConstraints;

    private Command drivingCommand;
    // private Command zeroHoodCommand;

    /**
     * @param swerveSubsystem   the swerve drive subsystem
     * @param flywheelSubsystem the flywheel subsystem (for zeroing the hood)
     */
    public AutoDriveUnderTrenchCommand(SwerveSubsystem swerveSubsystem, FlywheelSubsystem flywheelSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;

        this.pathfindingConstraints = new PathConstraints(
                TrenchConstants.MAX_VELOCITY,
                TrenchConstants.MAX_ACCELERATION,
                Units.degreesToRadians(TrenchConstants.MAX_ANGULAR_VELOCITY),
                Units.degreesToRadians(TrenchConstants.MAX_ANGULAR_ACCELERATION));

        addRequirements(swerveSubsystem, flywheelSubsystem);
    }

    @Override
    public void initialize() {
        String pathName = determinePathName();

        if (pathName == null) {
            DogLog.log("ADUT/Status", "Unable to determine path (no alliance data)");

            drivingCommand = Commands.none();
            // zeroHoodCommand = Commands.none();
        } else {
            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                boolean isRed = DriverStation.getAlliance()
                        .map(a -> a == Alliance.Red)
                        .orElse(false);
                PathPlannerPath displayPath = isRed ? path.flipPath() : path;

                DogLog.log("ADUT/Status", "Pathfinding then following " + pathName
                        + (isRed ? " (will be flipped for red by AutoBuilder)" : ""));

                swerveSubsystem.getSwerveDrive().field.getObject("TrenchPath")
                        .setPoses(displayPath.getPathPoses());

                Translation2d startPosition = displayPath.getPoint(0).position;
                Rotation2d startRotation = displayPath.getIdealStartingState() != null
                        ? displayPath.getIdealStartingState().rotation()
                        : Rotation2d.kZero;
                Pose2d startPose = new Pose2d(startPosition, startRotation);

                drivingCommand = Commands.sequence(
                        AutoBuilder.pathfindToPose(startPose, pathfindingConstraints),
                        AutoBuilder.followPath(path));

                // zeroHoodCommand = flywheelSubsystem.zeroHood();
            } catch (Exception e) {
                DogLog.log("ADUT/Status", "Failed to load path " + pathName + ": " + e.getMessage());
                DriverStation.reportError("ADUT: Failed to load path " + pathName, e.getStackTrace());

                drivingCommand = Commands.none();
                // zeroHoodCommand = Commands.none();
            }
        }

        drivingCommand.initialize();
        // zeroHoodCommand.initialize();
    }

    @Override
    public void execute() {
        drivingCommand.execute();

        // if (!zeroHoodCommand.isFinished()) {
        // zeroHoodCommand.execute();
        // } this needs to end the zero hood command once it is finished
    }

    @Override
    public void end(boolean interrupted) {
        drivingCommand.end(interrupted);
        // zeroHoodCommand.end(interrupted);

        swerveSubsystem.getSwerveDrive().field.getObject("TrenchPath").setPoses();
    }

    @Override
    public boolean isFinished() {
        return drivingCommand.isFinished();
    }

    private String determinePathName() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) {
            return null;
        }

        boolean isRed = alliance.get() == Alliance.Red;
        Translation2d hubPosition = Constants.HUB_LOCATION;
        Pose2d robotPose = swerveSubsystem.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        if (isRed) {
            Translation2d flippedRobotPos = FlippingUtil.flipFieldPosition(
                    new Translation2d(robotX, robotY));

            robotX = flippedRobotPos.getX();
            robotY = flippedRobotPos.getY();
        }

        boolean inOurZone = robotX < hubPosition.getX();
        boolean aboveHub = robotY > hubPosition.getY();

        String pathName;
        String zoneName;

        if (inOurZone && aboveHub) {
            pathName = TrenchConstants.PATH_B;
            zoneName = "Yellow->B";
        } else if (!inOurZone && aboveHub) {
            pathName = TrenchConstants.PATH_A;
            zoneName = "LightBlue->A";
        } else if (inOurZone && !aboveHub) {
            pathName = TrenchConstants.PATH_D;
            zoneName = "Green->D";
        } else {
            pathName = TrenchConstants.PATH_C;
            zoneName = "Purple->C";
        }

        DogLog.log("ADUT/Zone", zoneName);
        DogLog.log("ADUT/Path", pathName);
        DogLog.log("ADUT/RobotPose", swerveSubsystem.getPose().toString());
        DogLog.log("ADUT/IsRed", isRed);

        return pathName;
    }
}
