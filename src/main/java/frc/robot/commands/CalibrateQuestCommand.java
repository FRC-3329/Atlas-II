package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.CircleFitter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to calibrate the Quest headset's physical offset from the robot's center of rotation.
 * 
 * <p><b>How it works:</b></p>
 * <ol>
 *   <li>The robot rotates in place for a full 360 degree rotation at a controlled speed</li>
 *   <li>During rotation, the Quest's 3D position is recorded at each frame along with the robot's heading</li>
 *   <li>The collected Quest positions form a circular path around the robot's center of rotation</li>
 *   <li>A circle-fitting algorithm determines the center of this circle (the robot's center)</li>
 *   <li>For each data point, the vector from robot center to Quest is calculated and transformed into robot frame</li>
 *   <li>These vectors are averaged to produce the final Quest offset in robot coordinates</li>
 * </ol>
 */
public class CalibrateQuestCommand extends Command {
    
    /**
     * Record to store a Quest position sample with corresponding robot heading.
     * 
     * @param position The Quest's 3D position in world coordinates
     * @param robotHeading The robot's gyro heading when this sample was taken
     */
    private record QuestDataPoint(
        Translation2d position,
        Rotation2d robotHeading,
        double zPosition
    ) {}

    // Calibration parameters
    private static final AngularVelocity ROTATION_SPEED = DegreesPerSecond.of(60);
    private static final int MIN_DATA_POINTS = 100;
    private static final double ROTATION_THRESHOLD_DEGREES = 350.0;
    
    // SmartDashboard keys
    private static final String STATUS_KEY = "QuestCalibration/Status";
    private static final String OFFSET_X_KEY = "QuestCalibration/OffsetX";
    private static final String OFFSET_Y_KEY = "QuestCalibration/OffsetY";
    private static final String OFFSET_Z_KEY = "QuestCalibration/OffsetZ";
    private static final String DATA_POINTS_KEY = "QuestCalibration/DataPoints";

    private final SwerveSubsystem swerve;
    private final QuestNavSubsystem questNav;
    
    private final List<QuestDataPoint> collectedPoints = new ArrayList<>();
    private Rotation2d initialHeading;

    /**
     * Creates a new Quest calibration command.
     * 
     * @param swerve Swerve drive subsystem
     * @param questNav Quest subsystem 
     */
    public CalibrateQuestCommand(SwerveSubsystem swerve, QuestNavSubsystem questNav) {
        addRequirements(swerve, questNav);

        this.swerve = swerve;
        this.questNav = questNav;
    }

    @Override
    public void initialize() {
        collectedPoints.clear();
        initialHeading = swerve.getSwerveDrive().getYaw();
        
        SmartDashboard.putString(STATUS_KEY, "Initializing...");
        SmartDashboard.putNumber(DATA_POINTS_KEY, 0);
        
        if (!questNav.isConnected()) {
            DriverStation.reportWarning(
                "Quest calibration started but Quest is not connected :(",
                false
            );
        }
        if (!questNav.isTracking()) {
            DriverStation.reportWarning(
                "Quest calibration started but Quest is not tracking :(",
                false
            );
        }
    }

    @Override
    public void execute() {
        // Rotate robot in place at constant speed
        swerve.driveFieldOriented(
            new ChassisSpeeds(0, 0, ROTATION_SPEED.in(RadiansPerSecond))
        );

        // Attempt to collect Quest pose data
        Optional<Pose3d> questPoseOpt = questNav.getQuestPoseRaw();
        
        if (questPoseOpt.isPresent()) {
            Pose3d questPose = questPoseOpt.get();

            Translation2d questPosition2d = new Translation2d(
                questPose.getX(),
                questPose.getY()
            );

            double zPosition = questPose.getZ();
            Rotation2d currentHeading = swerve.getSwerveDrive().getYaw();
            
            collectedPoints.add(new QuestDataPoint(questPosition2d, currentHeading, zPosition));
        }

        SmartDashboard.putString(
            STATUS_KEY,
            String.format(
                "Collecting data: %d points (%.1f° rotation)",
                collectedPoints.size(),
                Math.abs(
                    swerve
                        .getSwerveDrive()
                        .getYaw()
                        .minus(initialHeading)
                        .getDegrees()
                )
            )
        );
        SmartDashboard.putNumber(DATA_POINTS_KEY, collectedPoints.size());
    }

    @Override
    public boolean isFinished() {
        /*
            Finishes after completing nearly a full rotation
            Use 350 degree threshold to ensure completion while allowing for some margin
        */
        double rotationDegrees = Math.abs(
            swerve
                .getSwerveDrive()
                .getYaw()
                .minus(initialHeading)
                .getDegrees()
        );

        return rotationDegrees > ROTATION_THRESHOLD_DEGREES;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop robot rotation
        swerve.driveFieldOriented(
            new ChassisSpeeds(0, 0, 0)
        );

        if (interrupted) {
            handleInterrupt();
            return;
        }

        if (collectedPoints.size() < MIN_DATA_POINTS) {
            handleInsufficient();
            return;
        }

        calculate();
    }

    private void handleInterrupt() {
        DriverStation.reportWarning("Quest calibration interrupted, data discarded.", false);
        SmartDashboard.putString(STATUS_KEY, "Interrupted");
    }

    private void handleInsufficient() {
        String error = String.format(
            "Quest calibration failed: Insufficient data points (%d/%d). " +
            "Ensure Quest is connected and tracking throughout the rotation.",
            collectedPoints.size(),
            MIN_DATA_POINTS
        );
    
        DriverStation.reportError(error, false);
        SmartDashboard.putString(STATUS_KEY, "Failed, Insufficient data");
    }

    private void calculate() {
        SmartDashboard.putString(STATUS_KEY, "Processing data...");

        // Extract 2D Quest pos for circle fitting
        List<Translation2d> questPositions = collectedPoints.stream().map(QuestDataPoint::position).toList();

        // Fit a circle to find the robot's center of rotation
        Optional<Translation2d> robotCenterOpt = CircleFitter.fit(questPositions);

        if (robotCenterOpt.isEmpty()) {
            handleFailure();
            return;
        }

        Translation2d robotCenterInWorld = robotCenterOpt.get();

        // Calculate Quest offset in robot frame for each point and average
        double sumOffsetX = 0.0;
        double sumOffsetY = 0.0;
        double sumOffsetZ = 0.0;

        for (QuestDataPoint dataPoint : collectedPoints) {
            // Vector from robot center to Quest in world frame
            Translation2d offsetInWorld = dataPoint.position().minus(robotCenterInWorld);

            // Transform to robot frame by rotating by inverse of robot heading
            Translation2d offsetInRobot = offsetInWorld.rotateBy(
                dataPoint.robotHeading().unaryMinus()
            );

            sumOffsetX += offsetInRobot.getX();
            sumOffsetY += offsetInRobot.getY();
            sumOffsetZ += dataPoint.zPosition();
        }

        // Calculate average offset
        int numPoints = collectedPoints.size();
        double avgOffsetX = sumOffsetX / numPoints;
        double avgOffsetY = sumOffsetY / numPoints;
        double avgOffsetZ = sumOffsetZ / numPoints;

        publishResults(avgOffsetX, avgOffsetY, avgOffsetZ);
    }

    private void handleFailure() {
        DriverStation.reportError(
            "Quest calibration failed: Circle fitting could not converge.", 
            false
        );
        SmartDashboard.putString(STATUS_KEY, "Failed, Circle fit error");
    }

    /**
     * Publishes the calculated calibration results to SmartDashboard and DriverStation.
     * 
     * @param offsetX X offset in meters (forward/backward in robot frame)
     * @param offsetY Y offset in meters (left/right in robot frame)
     * @param offsetZ Z offset in meters (up/down)
     */
    private void publishResults(double offsetX, double offsetY, double offsetZ) {
        SmartDashboard.putString(STATUS_KEY, "Complete!");
        SmartDashboard.putNumber(OFFSET_X_KEY, offsetX);
        SmartDashboard.putNumber(OFFSET_Y_KEY, offsetY);
        SmartDashboard.putNumber(OFFSET_Z_KEY, offsetZ);

        String result = String.format(
            "Quest calibration complete! Offset: X=%.4fm, Y=%.4fm, Z=%.4fm (%d points)",
            offsetX, offsetY, offsetZ, collectedPoints.size()
        );

        DriverStation.reportWarning(result, false);
    }
}
