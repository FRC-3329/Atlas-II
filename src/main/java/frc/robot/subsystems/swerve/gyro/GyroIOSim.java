package frc.robot.subsystems.swerve.gyro;

// https://github.com/pittsfordrobotics/ChargedUp2023/blob/master/src/main/java/com/team3181/frc2023/subsystems/swerve/GyroIOSim.java
public class GyroIOSim implements GyroIO {
    private double yawPositionRad = 0.0;
    private double yawVelocityRadPerSec = 0.0;

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPositionRad = yawPositionRad;
        inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
        inputs.pitchPositionRad = 0;
        inputs.rollPositionRad = 0;
    }

    @Override
    public void zeroGyro() {
        yawPositionRad = 0.0;
        yawVelocityRadPerSec = 0.0;
    }

    public void setYaw(double yawPositionRad, double yawVelocityRadPerSec) {
        this.yawPositionRad = yawPositionRad;
        this.yawVelocityRadPerSec = yawVelocityRadPerSec;
    }
}
