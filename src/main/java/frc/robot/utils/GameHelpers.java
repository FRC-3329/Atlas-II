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
 * Computes game-state-aware targeting: selects the correct target based
 * on field position, applies shoot-on-the-move offsets, and tracks
 * phase shift timing.
 */
public class GameHelpers extends SubsystemBase {
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelocitySupplier;
    private final MutDistance virtualTargetDistance;
    private Rotation2d virtualTargetFieldAngle;
    private Translation2d virtualTargetTranslation = Constants.HUB_LOCATION;
    private String cachedGameData = "";
    private ShotParameters.Parameters shotParameters;

    public GameHelpers(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotVelocitySupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.virtualTargetDistance = Meters.mutable(0.0);
    }

    /** Returns the hub position, flipped for red alliance. */
    public Translation2d calculateHubPosition() {
        Translation2d hubPosition = Constants.HUB_LOCATION;
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            hubPosition = FlippingUtil.flipFieldPosition(hubPosition);
        }

        return hubPosition;
    }

    public Distance getHubDistance() {
        Translation2d robotTranslation = robotPoseSupplier.get().getTranslation();
        double distanceMeters = robotTranslation.getDistance(calculateHubPosition());
        return Meters.of(distanceMeters);
    }

    public double getHubDistanceMeters() {
        Translation2d robotTranslation = robotPoseSupplier.get().getTranslation();
        return robotTranslation.getDistance(calculateHubPosition());
    }

    /**
     * Angle and distance to the SOTM-adjusted virtual target,
     * accounting for robot velocity so the fuel arrives at the
     * real target despite robot motion.
     */
    public Rotation2d getVirtualTargetFieldAngle() {
        return virtualTargetFieldAngle;
    }

    public Translation2d getVirtualTargetTranslation() {
        return virtualTargetTranslation;
    }

    public Distance getVirtualTargetDistance() {
        return virtualTargetDistance;
    }

    public ShotParameters.Parameters getShotParameters() {
        return shotParameters;
    }

    public Rotation2d getAngleToHub() {
        Translation2d robotTranslation = robotPoseSupplier.get().getTranslation();
        return calculateHubPosition().minus(robotTranslation).getAngle();
    }

    public Optional<Alliance> getAlliance() {
        return DriverStation.getAlliance();
    }

    public boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    public boolean isBlueAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue;
    }

    public Pose2d getRobotPose() {
        return robotPoseSupplier.get();
    }

    public ChassisSpeeds getRobotVelocites() {
        return robotVelocitySupplier.get();
    }

    /**
     * Determines which alliance's goal is currently scoreable based on
     * the phase shift schedule encoded in game data.
     * <p>
     * During auto, the team's own goal is always active.
     * During teleop, goals alternate on a fixed schedule; the game data
     * string tells us which alliance goes inactive first.
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

    public boolean isAllianceGoalActive() {
        Optional<Alliance> activeAlliance = getActiveGoalAlliance();
        Optional<Alliance> teamAlliance = DriverStation.getAlliance();

        if (activeAlliance.isEmpty() || teamAlliance.isEmpty()) {
            return false;
        }

        return activeAlliance.get() == teamAlliance.get();
    }

    /**
     * The field is divided at the hub X coordinate.
     * Blue zone = X < hub; Red zone = X > hub (after flipping).
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

    public boolean isAboveHub() {
        Pose2d robotPose = getRobotPose();
        Translation2d hubPosition = calculateHubPosition();

        return robotPose.getY() > hubPosition.getY();
    }

    public boolean isValidShotDistance() {
        double distanceToHub = virtualTargetDistance.in(Meters);

        return distanceToHub >= Constants.MIN_SHOT_DISTANCE
                && distanceToHub <= Constants.MAX_SHOT_DISTANCE;
    }

    /**
     * Phase shift boundaries: 130, 105, 80, 55, and 30 seconds remaining.
     * Match time counts down from ~135.
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

    /** Returns true in the 4s before a phase shift so the driver can prepare. */
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
            // In opponent zone and above hub - aim at top free space, shifted into our
            // alliance zone
            double xOffset = isRedAlliance() ? TurretConstants.PASS_X_OFFSET : -TurretConstants.PASS_X_OFFSET;
            targetTranslation = new Translation2d(
                    hubTranslation.getX() + xOffset,
                    hubTranslation.getY() + TurretConstants.TOP_FREE_SPACE_Y_OFFSET);

            DogLog.log((getName() + "/TargetZone"), "OpponentZoneTop");
        } else {
            // In opponent zone and below hub - aim at bottom free space, shifted into our
            // alliance zone
            double xOffset = isRedAlliance() ? TurretConstants.PASS_X_OFFSET : -TurretConstants.PASS_X_OFFSET;
            targetTranslation = new Translation2d(
                    hubTranslation.getX() + xOffset,
                    hubTranslation.getY() - TurretConstants.BOTTOM_FREE_SPACE_Y_OFFSET);

            DogLog.log((getName() + "/TargetZone"), "OpponentZoneBottom");
        }

        // Compute the SOTM-adjusted virtual target and shot parameters
        ShootOnTheMove.Shot shot = ShootOnTheMove.calculate(getRobotPose(), getRobotVelocites(),
                pose -> ShotParameters.getShotParameters(pose.transformBy(TurretConstants.ROBOT_TO_TURRET)
                        .getTranslation().getDistance(targetTranslation)));
        this.shotParameters = shot.parameters();

        Translation2d currentTurretTranslation = getRobotPose().transformBy(TurretConstants.ROBOT_TO_TURRET)
                .getTranslation();
        Translation2d futureTurretTranslation = shot.futureRobotPose().transformBy(TurretConstants.ROBOT_TO_TURRET)
                .getTranslation();

        // Virtual target = real target shifted opposite to robot motion,
        // so aiming at it from the current position accounts for where
        // the robot will be when the fuel arrives.
        Translation2d virtualTargetTranslation = targetTranslation
                .minus(futureTurretTranslation.minus(currentTurretTranslation));

        // When the robot is stationary, no SOTM adjustment is needed
        if (currentTurretTranslation.equals(futureTurretTranslation)) {
            this.virtualTargetTranslation = targetTranslation;
            this.virtualTargetFieldAngle = targetTranslation.minus(currentTurretTranslation).getAngle();
            this.virtualTargetDistance.mut_replace(targetTranslation.getDistance(currentTurretTranslation), Meters);
        } else {
            this.virtualTargetTranslation = virtualTargetTranslation;
            this.virtualTargetFieldAngle = virtualTargetTranslation.minus(currentTurretTranslation).getAngle();
            this.virtualTargetDistance.mut_replace(virtualTargetTranslation.getDistance(currentTurretTranslation),
                    Meters);
        }

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
