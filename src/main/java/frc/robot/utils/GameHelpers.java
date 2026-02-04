package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants;

/**
 * Utility class for game-specific operations.
 */
public class GameHelpers {
    private final Supplier<Pose2d> robotPoseSupplier;

    /**
     * @param robotPoseSupplier A supplier that provides the robot's current pose on the field
     */
    public GameHelpers(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
    }

    /**
     * @return The hub position in the current alliance's coordinate system
     */
    public Translation2d calculateHubPosition() {
        Translation2d hubPosition = Constants.HUB_LOCATION;
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            hubPosition = FlippingUtil.flipFieldPosition(hubPosition);
        }
        
        return hubPosition;
    }

    /**
     * @return The distance to the hub in meters
     */
    public Distance getHubDistance() {
        Translation2d robotTranslation = robotPoseSupplier.get().getTranslation();
        double distanceMeters = robotTranslation.getDistance(calculateHubPosition());
        return Meters.of(distanceMeters);
    }

    /**
     * @return The distance to the hub in meters
     */
    public double getHubDistanceMeters() {
        Translation2d robotTranslation = robotPoseSupplier.get().getTranslation();
        return robotTranslation.getDistance(calculateHubPosition());
    }

    /**
     * This is the angle from the robot's position to the hub, relative to the field's x-axis.
     * 
     * @return The angle to the hub as a Rotation2d
     */
    public Rotation2d getAngleToHub() {
        Translation2d robotTranslation = robotPoseSupplier.get().getTranslation();
        return calculateHubPosition().minus(robotTranslation).getAngle();
    }

    /**
     * @return current alliance (Red or Blue) if present
     */
    public Optional<Alliance> getAlliance() {
        return DriverStation.getAlliance();
    }

    /**
     * @return true if the alliance is Red, false otherwise
     */
    public boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    /**
     * @return true if the alliance is Blue, false otherwise
     */
    public boolean isBlueAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue;
    }

    /**
     * @return The robot's current pose on the field
     */
    public Pose2d getRobotPose() {
        return robotPoseSupplier.get();
    }
}
