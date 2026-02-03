package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

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
     * Gets the distance from the robot to the hub.
     * 
     * @return The distance to the hub in meters (as a Distance unit)
     */
    public Distance getHubDistance() {
        Translation2d robotTranslation = robotPoseSupplier.get().getTranslation();
        double distanceMeters = robotTranslation.getDistance(Constants.HUB_LOCATION);
        return Meters.of(distanceMeters);
    }

    /**
     * Gets the distance from the robot to the hub as a raw double value.
     * 
     * @return The distance to the hub in meters
     */
    public double getHubDistanceMeters() {
        Translation2d robotTranslation = robotPoseSupplier.get().getTranslation();
        return robotTranslation.getDistance(Constants.HUB_LOCATION);
    }

    /**
     * Gets the angle from the robot to the hub.
     * This is the angle from the robot's position to the hub, relative to the field's x-axis.
     * 
     * @return The angle to the hub as a Rotation2d
     */
    public Rotation2d getAngleToHub() {
        Translation2d robotTranslation = robotPoseSupplier.get().getTranslation();
        // Calculate the vector from robot to hub, then get its angle
        return Constants.HUB_LOCATION.minus(robotTranslation).getAngle();
    }

    /**
     * Gets the current alliance color.
     * 
     * @return The current alliance (Red or Blue) if present, otherwise null
     */
    public Alliance getAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() : null;
    }

    /**
     * Checks if the current alliance is Red.
     * 
     * @return true if the alliance is Red, false otherwise
     */
    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    /**
     * Checks if the current alliance is Blue.
     * 
     * @return true if the alliance is Blue, false otherwise
     */
    public boolean isBlueAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue;
    }

    /**
     * Gets the robot's current pose.
     * 
     * @return The robot's current pose on the field
     */
    public Pose2d getRobotPose() {
        return robotPoseSupplier.get();
    }
}
