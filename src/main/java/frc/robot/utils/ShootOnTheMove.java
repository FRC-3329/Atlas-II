package frc.robot.utils;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootOnTheMove {
    private static final int MAX_ITERATIONS = 3; // 2-5 iterations should suffice

    /** The shot parameters at the virtual target. */
    public record Shot(ShotParameters.Parameters parameters, Pose2d virtualTarget) {
    }

    /**
     * Dynamic Shooting via Time-of-Flight Recursion
     * 
     * @param currentPose        The robot's current pose on the field
     * @param chassisSpeeds      The robot relative velocities
     * @param parametersFunction a function to convert the position of the robot on
     *                           the field into shot parameters
     * @return A shot condition with the shot parameters and the virtual target
     *         being aimed at
     */
    public static Shot calculate(
            final Pose2d currentPose,
            final ChassisSpeeds chassisSpeeds,
            final Function<Pose2d, ShotParameters.Parameters> parametersFunction) {
        Pose2d virtualTarget = currentPose;

        ShotParameters.Parameters parameters = parametersFunction.apply(currentPose);
        double timeOfFlightSeconds = parameters.tofSeconds();

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            final Twist2d twist = new Twist2d(
                    chassisSpeeds.vxMetersPerSecond * timeOfFlightSeconds,
                    chassisSpeeds.vyMetersPerSecond * timeOfFlightSeconds,
                    chassisSpeeds.omegaRadiansPerSecond * timeOfFlightSeconds);

            virtualTarget = currentPose.exp(twist);
            parameters = parametersFunction.apply(virtualTarget);
            timeOfFlightSeconds = parameters.tofSeconds();
        }

        return new Shot(parameters, virtualTarget);
    }
}
