package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.OrientToHubConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GameHelpers;

/**
 * Orients the robot so the turret faces the hub.
 * The turret is rear-mounted, so the robot must face 180° away from the hub.
 */
public class OrientToHubCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final GameHelpers gameHelpers;
    private final ProfiledPIDController rotationController;

    public OrientToHubCommand(SwerveSubsystem swerveSubsystem, GameHelpers gameHelpers) {
        this.swerveSubsystem = swerveSubsystem;
        this.gameHelpers = gameHelpers;
        this.rotationController = new ProfiledPIDController(
                OrientToHubConstants.ORIENT_TO_HUB_KP,
                OrientToHubConstants.ORIENT_TO_HUB_KI,
                OrientToHubConstants.ORIENT_TO_HUB_KD,
                new TrapezoidProfile.Constraints(
                        OrientToHubConstants.ORIENT_TO_HUB_MAX_VELOCITY.in(RadiansPerSecond),
                        OrientToHubConstants.ORIENT_TO_HUB_MAX_ACCELERATION
                                .in(RadiansPerSecondPerSecond)));

        // Wrap around ±π so the PID takes the shortest rotation path
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(
                OrientToHubConstants.ORIENT_TO_HUB_TOLERANCE.in(edu.wpi.first.units.Units.Radians));

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Rotation2d angleToHub = gameHelpers.getAngleToHub();
        Rotation2d targetAngle = angleToHub.plus(Rotation2d.fromDegrees(180.0));
        Rotation2d currentHeading = swerveSubsystem.getSwerveDrive().getPose().getRotation();

        rotationController.reset(currentHeading.getRadians());
        rotationController.setGoal(targetAngle.getRadians());
    }

    @Override
    public void execute() {
        Rotation2d currentHeading = swerveSubsystem.getSwerveDrive().getPose().getRotation();
        double rotationSpeed = rotationController.calculate(currentHeading.getRadians());
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, rotationSpeed);

        swerveSubsystem.driveFieldOriented(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.driveFieldOriented(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return rotationController.atGoal();
    }
}
