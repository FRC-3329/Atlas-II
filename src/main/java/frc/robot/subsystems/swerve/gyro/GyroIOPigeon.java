package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.SwerveConstants;

// https://github.com/pittsfordrobotics/ChargedUp2023/blob/master/src/main/java/com/team3181/frc2023/subsystems/swerve/GyroIOPigeon.java
public class GyroIOPigeon implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(SwerveConstants.Pigeon.ID);
    private final Pigeon2Configuration config = new Pigeon2Configuration();

    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<Angle> pitch = pigeon.getPitch();
    private final StatusSignal<Angle> roll = pigeon.getRoll();
    private final StatusSignal<AngularVelocity> rollVelocity = pigeon.getAngularVelocityXWorld();
    private final StatusSignal<AngularVelocity> pitchVelocity = pigeon.getAngularVelocityYWorld();
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

    public GyroIOPigeon() {
        config.MountPose.MountPoseRoll = 0;
        pigeon.getConfigurator().apply(config);
        pigeon.setYaw(0);
    }

    @Override
    public void zeroGyro() {
        pigeon.setYaw(0);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                yaw,
                pitch,
                roll,
                rollVelocity,
                pitchVelocity,
                yawVelocity).isOK();

        inputs.yawPositionRad = Units.degreesToRadians(yaw.getValueAsDouble()); // ccw+
        inputs.pitchPositionRad = Units.degreesToRadians(-pitch.getValueAsDouble()); // up+ down-
        inputs.rollPositionRad = Units.degreesToRadians(roll.getValueAsDouble()); // cw+
        inputs.rollVelocityRadPerSec = Units.degreesToRadians(rollVelocity.getValueAsDouble()); // cw+
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-pitchVelocity.getValueAsDouble()); // up+ down-
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble()); // ccw+
    }
}
