package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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

        // We only need basic motor control (status 0)
        // Set all telemetry frames (status 1-6) to 500ms to minimize CAN traffic
        config.signals
            .primaryEncoderVelocityPeriodMs(500)
            .primaryEncoderPositionPeriodMs(500)
            .analogVoltagePeriodMs(500)
            .analogVelocityPeriodMs(500)
            .analogPositionPeriodMs(500);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setDefaultCommand(
                this.runOnce(() -> {
                    motor.set(0);
                }).andThen(
                        this.idle()));
    }

    private void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        DogLog.log(getName() + "/Voltage", voltage, Volts);
    }

    public Command feed() {
        return this.run(() -> setVoltage(IndexerConstants.FEED_VOLTAGE))
                .withName("IndexerFeed");
    }

    @Override
    public void periodic() {
        DogLog.log(getName() + "/Current", motor.getOutputCurrent(), Amps);
        DogLog.log(getName() + "/Velocity", motor.getEncoder().getVelocity(), RPM);
    }
}
