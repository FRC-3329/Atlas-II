package frc.robot.subsystems.swerve.gyro;

// https://github.com/pittsfordrobotics/ChargedUp2023/blob/master/src/main/java/com/team3181/frc2023/subsystems/swerve/GyroIOSim.java
public class GyroIOSim implements GyroIO {
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = false;
        inputs.yawPositionRad = 0;
        inputs.yawVelocityRadPerSec = 0;
        inputs.pitchPositionRad = 0;
        inputs.rollPositionRad = 0;
    }
}