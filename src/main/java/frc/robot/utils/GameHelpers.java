package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.util.FlippingUtil;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;

/**
 * Utility class for game-specific operations.
 */
public class GameHelpers extends SubsystemBase {
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelocitySupplier;
    private final MutDistance virtualTargetDistance;
    private Rotation2d virtualTargetFieldAngle;
    private String cachedGameData = "";
    private ShotParameters.Parameters shotParameters;

    /**
     * @param robotPoseSupplier     A supplier that provides the robot's current
     *                              pose on
     *                              the field
     * @param robotVelocitySupplier A supplier to provide the current robot centric
     *                              velocity
     */
    public GameHelpers(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotVelocitySupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.virtualTargetDistance = Meters.mutable(0.0);
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
     * @return The angle between the robot and the target depending on the robot's
     *         current position on the field and velocity. The current position
     *         selects the target while the position and velocity adjust the virtual
     *         target for SOTM.
     */
    public Rotation2d getVirtualTargetFieldAngle() {
        return virtualTargetFieldAngle;
    }

    /**
     * @return The distance between the robot and the target depending on the
     *         robot's current position on the field and velocity. The current
     *         position selects the target while the position and velocity adjust
     *         the virtual target for SOTM.
     */
    public Distance getVirtualTargetDistance() {
        return virtualTargetDistance;
    }

    /**
     * @return The updated shot parameters for the robot's current state on the
     *         field. This uses the position and velocity to determine both the
     *         correct target and offset the target to account for SOTM.
     */
    public ShotParameters.Parameters getShotParameters() {
        return shotParameters;
    }

    /**
     * This is the angle from the robot's position to the hub, relative to the
     * field's x-axis.
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
     * @return The robot's current velocity (not field velocity)
     */
    public ChassisSpeeds getRobotVelocites() {
        return robotVelocitySupplier.get();
    }

    /**
     * Determines which alliance's goal is currently active based on match time and
     * game data.
     * The game data specifies which alliance's goal goes inactive first.
     * During auto, the team's own goal is always active.
     * 
     * @return The active Alliance (Red or Blue), or empty if game data is
     *         unavailable
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
     * @return true if the team's goal is active, false otherwise or if game data is
     *         unavailable
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

    /**
     * Teleop match time counts down from ~135 to 0.
     * Phase shift boundaries are at 130, 105, 80, 55, and 30 seconds.
     * 
     * @return seconds remaining in the current shift
     */
    public int timeLeftInShiftSeconds() {
        double currentMatchTime = DriverStation.getMatchTime();

        if (currentMatchTime >= 130) {
            return (int) (currentMatchTime - 130);
        } else if (currentMatchTime >= 105) {
            return (int) (currentMatchTime - 105);
        } else if (currentMatchTime >= 80) {
            return (int) (currentMatchTime - 80);
        } else if (currentMatchTime >= 55) {
            return (int) (currentMatchTime - 55);
        } else if (currentMatchTime >= 30) {
            return (int) (currentMatchTime - 30);
        } else {
            return (int) currentMatchTime;
        }
    }

    /**
     * @return true if a phase shift is about to happen within the next 4 seconds
     */
    public boolean isPhaseShiftImminent() {
        int timeLeft = timeLeftInShiftSeconds();
        return timeLeft <= 4 && timeLeft > 0;
    }

    @Override
    public void periodic() {
        // calculate proper target so we don't fire at the hub if we are not in our zone
        Translation2d targetTranslation;
        Translation2d hubTranslation = calculateHubPosition();

        if (isInOurZone()) {
            // In our alliance zone - aim directly at hub center
            targetTranslation = hubTranslation;
            DogLog.log((getName() + "/TargetZone"), "OurZone");
        } else if (isAboveHub()) {
            // In opponent zone and above hub - aim at top free space
            targetTranslation = new Translation2d(
                    hubTranslation.getX(),
                    hubTranslation.getY() + TurretConstants.TOP_FREE_SPACE_Y_OFFSET);

            DogLog.log((getName() + "/TargetZone"), "OpponentZoneTop");
        } else {
            // In opponent zone and below hub - aim at bottom free space
            targetTranslation = new Translation2d(
                    hubTranslation.getX(),
                    hubTranslation.getY() - TurretConstants.BOTTOM_FREE_SPACE_Y_OFFSET);

            DogLog.log((getName() + "/TargetZone"), "OpponentZoneBottom");
        }

        // now that we have the target location, calculate the virtual target and shot
        // parameters
        ShootOnTheMove.Shot shot = ShootOnTheMove.calculate(getRobotPose(), getRobotVelocites(),
                pose -> ShotParameters.getShotParameters(pose.getTranslation().getDistance(targetTranslation)));
        this.shotParameters = shot.parameters(); // store for flywheel to use

        // get the current and future robot translations
        Translation2d currentRobotTranslation = getRobotPose().getTranslation();
        Translation2d futureRobotTranslation = shot.futureRobotPose().getTranslation();

        // Virtual target is the target translation minus the difference between the
        // future and current robot translation. The order of subtractions here matters.
        Translation2d virtualTargetTranslation = targetTranslation
                .minus(futureRobotTranslation.minus(currentRobotTranslation));

        // store distance/angle to virtual target
        // if the future is equal to the current, then just use current
        if (currentRobotTranslation.equals(futureRobotTranslation)) {
            this.virtualTargetFieldAngle = targetTranslation.minus(currentRobotTranslation).getAngle();
            this.virtualTargetDistance.mut_replace(targetTranslation.getDistance(currentRobotTranslation), Meters);
        } else {
            this.virtualTargetFieldAngle = virtualTargetTranslation.minus(currentRobotTranslation).getAngle();
            this.virtualTargetDistance.mut_replace(virtualTargetTranslation.getDistance(currentRobotTranslation),
                    Meters);
        }

        // logging
        DogLog.log((getName() + "/InOurZone"), isInOurZone());
        DogLog.log((getName() + "/AboveHub"), isAboveHub());
        DogLog.log((getName() + "/VirtualTargetFieldAngle"), virtualTargetFieldAngle);
        DogLog.log((getName() + "/VirtualTargetDistance"), virtualTargetDistance);
        DogLog.log((getName() + "/SelectedTargetPosition"), new Pose2d(targetTranslation, Rotation2d.kZero));
        DogLog.log((getName() + "/VirtualTargetPosition"),
                new Pose2d(virtualTargetTranslation, virtualTargetFieldAngle));
        DogLog.log((getName() + "/FutureRobotPosition"), shot.futureRobotPose());
        DogLog.forceNt.log((getName() + "/ShiftTimeLeft"), timeLeftInShiftSeconds());
        DogLog.forceNt.log((getName() + "/IsAllianceGoalActive"), isAllianceGoalActive());
        DogLog.forceNt.log((getName() + "/PhaseShiftImminent"), isPhaseShiftImminent());
    }
}
