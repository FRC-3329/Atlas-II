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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final Trigger stallDetected;

    public IndexerSubsystem() {
        motor = new SparkMax(IndexerConstants.MOTOR_ID, MotorType.kBrushless);
        stallDetected = new Trigger(() -> motor.getOutputCurrent() >= (IndexerConstants.CURRENT_LIMIT - 1.0))
                .debounce(IndexerConstants.CURRENT_STALL_TIME);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(IndexerConstants.CURRENT_LIMIT);
        config.inverted(IndexerConstants.INVERTED);
        config.idleMode(IndexerConstants.IDLE_MODE);

        config.signals
                .absoluteEncoderPositionAlwaysOn(false)
                .primaryEncoderVelocityAlwaysOn(false)
                .analogPositionAlwaysOn(false)
                .analogVelocityAlwaysOn(false)
                .externalOrAltEncoderPositionAlwaysOn(false)
                .externalOrAltEncoderVelocityAlwaysOn(false)
                .primaryEncoderPositionAlwaysOn(false)
                .primaryEncoderVelocityAlwaysOn(false)
                .iAccumulationAlwaysOn(false)
                .appliedOutputPeriodMs(20)
                .faultsPeriodMs(20);

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

    public Command feedBackwards() {
        return this.run(() -> setVoltage(-IndexerConstants.FEED_VOLTAGE))
                .withName("IndexerFeedBackwards");
    }

    public Command smartFeed() {
        return Commands.repeatingSequence(
                feed().until(stallDetected),
                feedBackwards().withTimeout(IndexerConstants.SMART_REVERSAL_TIME)).withName("IndexerSmartFeed");
    }

    @Override
    public void periodic() {
        DogLog.log(getName() + "/Current", motor.getOutputCurrent(), Amps);
        DogLog.log(getName() + "/Velocity", motor.getEncoder().getVelocity(), RPM);
        DogLog.log(getName() + "/stallDetected", stallDetected.getAsBoolean());
    }
}
