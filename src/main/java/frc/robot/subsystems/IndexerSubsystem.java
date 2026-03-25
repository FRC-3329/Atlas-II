package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
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
    private final SparkMax indexerMotor;
    private final SparkMax beltMotor;
    private final Trigger stallDetected;

    public IndexerSubsystem() {
        indexerMotor = new SparkMax(IndexerConstants.Indexer.MOTOR_ID, MotorType.kBrushless);
        stallDetected = new Trigger(
                () -> indexerMotor.getOutputCurrent() >= (IndexerConstants.Indexer.CURRENT_LIMIT - 1.0))
                .debounce(IndexerConstants.CURRENT_STALL_TIME);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(IndexerConstants.Indexer.CURRENT_LIMIT);
        config.inverted(IndexerConstants.Indexer.INVERTED);
        config.idleMode(IndexerConstants.Indexer.IDLE_MODE);

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

        indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure belt motor to follow indexer motor
        beltMotor = new SparkMax(IndexerConstants.Belt.MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig beltConfig = new SparkMaxConfig();
        beltConfig.follow(indexerMotor);
        beltConfig.smartCurrentLimit(IndexerConstants.Belt.CURRENT_LIMIT);
        beltConfig.idleMode(IndexerConstants.Belt.IDLE_MODE);

        beltConfig.signals
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

        beltMotor.configure(beltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setDefaultCommand(
                this.runOnce(() -> {
                    indexerMotor.set(0);
                }).andThen(
                        this.idle()));
    }

    private void setVoltage(double voltage) {
        indexerMotor.setVoltage(voltage);
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

    public Command stop() {
        return this.runOnce(() -> indexerMotor.stopMotor());
    }

    @Override
    public void periodic() {
        DogLog.log(getName() + "/Current", indexerMotor.getOutputCurrent(), Amps);
        DogLog.log(getName() + "/Velocity", indexerMotor.getEncoder().getVelocity(), RPM);
        DogLog.log(getName() + "/stallDetected", stallDetected.getAsBoolean());

        // Log temp when current limit is high enough to cause heat concerns
        if (IndexerConstants.Indexer.CURRENT_LIMIT >= 60) {
            DogLog.log(getName() + "/Temperature", indexerMotor.getMotorTemperature(), Celsius);
        }

        DogLog.log(getName() + "/BeltCurrent", beltMotor.getOutputCurrent(), Amps);
        DogLog.log(getName() + "/BeltVelocity", beltMotor.getEncoder().getVelocity(), RPM);
    }
}
