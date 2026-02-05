package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final SparkMax motor;

    public IndexerSubsystem() {
        motor = new SparkMax(IndexerConstants.MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(IndexerConstants.CURRENT_LIMIT);
        config.inverted(IndexerConstants.INVERTED);
        config.idleMode(IndexerConstants.IDLE_MODE);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setDefaultCommand(
                this.runOnce(() -> {
                    motor.set(0);
                }).andThen(
                        this.idle()));
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        DogLog.log(getName() + "/Voltage", voltage);
    }

    public Command feed() {
        return this.run(() -> setVoltage(IndexerConstants.FEED_VOLTAGE));
    }

    public Command idle() {
        return this.run(() -> {
        });
    }

    @Override
    public void periodic() {
        DogLog.log(getName() + "/Current", motor.getOutputCurrent());
        DogLog.log(getName() + "/Velocity", motor.getEncoder().getVelocity());
    }
}
