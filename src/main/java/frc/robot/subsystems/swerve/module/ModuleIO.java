package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public boolean driveConnected = false;
        public boolean angleConnected = false;
        public boolean encoderConnected = false;

        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelsius = 0.0;

        public double anglePositionRad = 0.0;
        public double angleVelocityRadPerSec = 0.0;
        public double angleAbsolutePositionRad = 0.0;
        public double angleAppliedVolts = 0.0;
        public double angleCurrentAmps = 0.0;
        public double angleTempCelsius = 0.0;
    }

    default void updateInputs(ModuleIOInputs inputs) {
    }

    default void setDesiredState(SwerveModuleState state) {
    }

    default void setDriveVoltage(double volts) {
    }

    default void setAngleVoltage(double volts) {
    }

    default void setBrakeMode(boolean brake) {
    }

    default void stop() {
    }
}
