package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Constants;
import frc.robot.constants.SwerveConstants;

public class ModuleIOSim implements ModuleIO {
    private final PIDController angleController = new PIDController(
            SwerveConstants.Simulation.kP,
            0.0,
            SwerveConstants.Simulation.kD);

    private double drivePositionMeters = 0.0;
    private double driveVelocityMetersPerSec = 0.0;
    private double anglePositionRad = 0.0;
    private double angleVelocityRadPerSec = 0.0;
    private double driveAppliedVolts = 0.0;
    private double angleAppliedVolts = 0.0;

    public ModuleIOSim() {
        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        drivePositionMeters += driveVelocityMetersPerSec * Constants.LOOP_TIME_SECONDS;
        angleVelocityRadPerSec = angleAppliedVolts;
        anglePositionRad += angleVelocityRadPerSec * Constants.LOOP_TIME_SECONDS;

        inputs.driveConnected = true;
        inputs.angleConnected = true;
        inputs.encoderConnected = true;
        inputs.drivePositionMeters = drivePositionMeters;
        inputs.driveVelocityMetersPerSec = driveVelocityMetersPerSec;
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveAppliedVolts) * 2.0;
        inputs.anglePositionRad = anglePositionRad;
        inputs.angleVelocityRadPerSec = angleVelocityRadPerSec;
        inputs.angleAbsolutePositionRad = anglePositionRad;
        inputs.angleAppliedVolts = angleAppliedVolts;
        inputs.angleCurrentAmps = Math.abs(angleAppliedVolts);
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        driveVelocityMetersPerSec = state.speedMetersPerSecond;

        driveAppliedVolts = MathUtil.clamp(
                state.speedMetersPerSecond / SwerveConstants.MAX_LINEAR_SPEED_METERS_PER_SEC * 12.0,
                -12.0,
                12.0);
        angleAppliedVolts = MathUtil.clamp(
                angleController.calculate(anglePositionRad, state.angle.getRadians()),
                -12.0,
                12.0);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveVelocityMetersPerSec = driveAppliedVolts / 12.0 * SwerveConstants.MAX_LINEAR_SPEED_METERS_PER_SEC;
    }

    @Override
    public void setAngleVoltage(double volts) {
        angleAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void stop() {
        setDesiredState(new SwerveModuleState(0.0, new Rotation2d(anglePositionRad)));
    }
}
