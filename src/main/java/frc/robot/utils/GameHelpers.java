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
    private String cachedGameData = "";

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

    /**
     * Determines which alliance's goal is currently active based on match time and game data.
     * The game data specifies which alliance's goal goes inactive first.
     * During auto, the team's own goal is always active.
     * 
     * @return The active Alliance (Red or Blue), or empty if game data is unavailable
     */
    public Optional<Alliance> getActiveGoalAlliance() {
        if (DriverStation.isAutonomous()) {
            return DriverStation.getAlliance();
        }

        double currentTime = DriverStation.getMatchTime();

        if (cachedGameData.isEmpty()) {
            cachedGameData = DriverStation.getGameSpecificMessage();

            if (cachedGameData.isEmpty()) {
                return Optional.empty();
            }
        }

        Alliance firstInactiveAlliance = cachedGameData.charAt(0) == 'R' ? Alliance.Red : Alliance.Blue;

        if (currentTime >= 130 || currentTime < 30) {
            return DriverStation.getAlliance();
        } else if (currentTime >= 105 || (currentTime < 80 && currentTime >= 55)) {
            return Optional.of(firstInactiveAlliance == Alliance.Red ? Alliance.Blue : Alliance.Red);
        } else {
            return Optional.of(firstInactiveAlliance);
        }
    }

    /**
     * @return true if the team's goal is active, false otherwise or if game data is unavailable
     */
    public boolean isAllianceGoalActive() {
        Optional<Alliance> activeAlliance = getActiveGoalAlliance();
        Optional<Alliance> teamAlliance = DriverStation.getAlliance();

        if (activeAlliance.isEmpty() || teamAlliance.isEmpty()) {
            return false;
        }

        return activeAlliance.get() == teamAlliance.get();
    }

    /**
     * Checks if the robot is in its own alliance zone.
     * The field is divided at the center line (X = half field width).
     * For blue alliance, our zone is X < center.
     * For red alliance, our zone is X > center.
     * 
     * @return true if robot is in its alliance's zone, false otherwise
     */
    public boolean isInOurZone() {
        Pose2d robotPose = getRobotPose();
        double robotX = robotPose.getX();
        Translation2d hubPosition = calculateHubPosition();
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) {
            return false;
        }
        
        if (alliance.get() == Alliance.Blue) {
            return robotX < hubPosition.getX();
        } else {
            return robotX > hubPosition.getX();
        }
    }

    /**
     * Checks if the robot is above the hub in Y position.
     * 
     * @return true if robot Y position is greater than hub Y position
     */
    public boolean isAboveHub() {
        Pose2d robotPose = getRobotPose();
        Translation2d hubPosition = calculateHubPosition();

        return robotPose.getY() > hubPosition.getY();
    }

    /**
     * Checks if the robot is within a valid shooting distance from the hub.
     * 
     * @return true if the robot is within the valid shooting distance range
     */
    public boolean isValidShotDistance() {
        double distanceToHub = getHubDistanceMeters();

        return distanceToHub >= Constants.MIN_SHOT_DISTANCE 
            && distanceToHub <= Constants.MAX_SHOT_DISTANCE;
    }
}
