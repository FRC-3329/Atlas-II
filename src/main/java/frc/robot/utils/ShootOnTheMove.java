package frc.robot.utils;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// Adapted from Team 1683 TechnoTitans' TitanWare2024
public class ShootOnTheMove {
    /** Converges quickly; diminishing returns beyond ~3 iterations. */
    private static final int MAX_ITERATIONS = 3;

    /**
     * Contains the converged shot parameters and the predicted robot pose
     * at the time the fuel reaches the target.
     */
    public record Shot(ShotParameters.Parameters parameters, Pose2d futureRobotPose) {
    }

    /**
     * Iteratively predicts where the robot will be when the fuel arrives
     * at the target, using time-of-flight to project future position,
     * then re-computing parameters from that position.
     */
    public static Shot calculate(
            final Pose2d currentPose,
            final ChassisSpeeds chassisSpeeds,
            final Function<Pose2d, ShotParameters.Parameters> parametersFunction) {
        Pose2d futureRobotPose = currentPose;

        ShotParameters.Parameters parameters = parametersFunction.apply(currentPose);
        double timeOfFlightSeconds = parameters.tofSeconds();

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            final Twist2d twist = new Twist2d(
                    chassisSpeeds.vxMetersPerSecond * timeOfFlightSeconds,
                    chassisSpeeds.vyMetersPerSecond * timeOfFlightSeconds,
                    chassisSpeeds.omegaRadiansPerSecond * timeOfFlightSeconds);

            futureRobotPose = currentPose.exp(twist);
            parameters = parametersFunction.apply(futureRobotPose);
            timeOfFlightSeconds = parameters.tofSeconds();
        }

        return new Shot(parameters, futureRobotPose);
    }
}
