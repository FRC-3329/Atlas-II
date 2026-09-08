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
import com.revrobotics.sim.SparkMaxSim;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.IndexerConstants;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;

public class IndexerSubsystem extends SubsystemBase {
    private final SparkMax indexerMotor;
    private final SparkMax beltMotor;
    private final Trigger stallDetected;
    private final SparkMaxSim indexerSparkSimulation;
    private final SparkMaxSim beltSparkSimulation;
    private final DCMotorSim indexerMechanismSimulation;
    private final DCMotorSim beltMechanismSimulation;

    private double indexerCommandedVoltage;
    private double beltCommandedVoltage;
    private volatile double simulationCurrentDrawAmps;

    public IndexerSubsystem() {
        indexerMotor = new SparkMax(IndexerConstants.Indexer.MOTOR_ID, MotorType.kBrushless);
        stallDetected = new Trigger(
                () -> indexerMotor.getOutputCurrent() >= (IndexerConstants.Indexer.CURRENT_LIMIT - 2.0))
                .debounce(IndexerConstants.CURRENT_STALL_TIME);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(IndexerConstants.Indexer.CURRENT_LIMIT);
        config.inverted(IndexerConstants.Indexer.INVERTED);
        config.idleMode(IndexerConstants.Indexer.IDLE_MODE);

        // Disable unused CAN frames to reduce bus utilization
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

        // Belt follows the indexer motor so they always spin together
        beltMotor = new SparkMax(IndexerConstants.Belt.MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig beltConfig = new SparkMaxConfig();
        beltConfig.smartCurrentLimit(IndexerConstants.Belt.CURRENT_LIMIT);
        beltConfig.inverted(IndexerConstants.Belt.INVERTED);
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

        if (RobotBase.isSimulation()) {
            DCMotor neo = DCMotor.getNEO(1);
            indexerSparkSimulation = new SparkMaxSim(indexerMotor, neo);
            beltSparkSimulation = new SparkMaxSim(beltMotor, neo);
            indexerMechanismSimulation = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            neo,
                            IndexerConstants.Indexer.MOMENT_OF_INERTIA,
                            1.0),
                    neo);
            beltMechanismSimulation = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            neo,
                            IndexerConstants.Belt.MOMENT_OF_INERTIA,
                            1.0),
                    neo);
            SimulatedBattery.addElectricalAppliances(
                    () -> edu.wpi.first.units.Units.Amps.of(simulationCurrentDrawAmps));
        } else {
            indexerSparkSimulation = null;
            beltSparkSimulation = null;
            indexerMechanismSimulation = null;
            beltMechanismSimulation = null;
        }

        setDefaultCommand(
                this.runOnce(() -> {
                    indexerCommandedVoltage = 0.0;
                    beltCommandedVoltage = 0.0;
                    indexerMotor.set(0);
                    beltMotor.set(0);
                }).andThen(
                        this.idle()));
    }

    private void setIndexerVoltage(double voltage) {
        indexerCommandedVoltage = voltage;
        indexerMotor.setVoltage(voltage);
        DogLog.log(getName() + "/Voltage", voltage, Volts);
    }

    private void setBeltVoltage(double voltage) {
        beltCommandedVoltage = voltage;
        beltMotor.setVoltage(voltage);
    }

    public Command feed() {
        return this.run(() -> {
            setIndexerVoltage(IndexerConstants.Indexer.FEED_VOLTAGE);
            setBeltVoltage(IndexerConstants.Belt.FEED_VOLTAGE);
        }).withName("IndexerFeed");
    }

    public Command feedBackwards() {
        return this.run(() -> {
            setIndexerVoltage(-IndexerConstants.Indexer.FEED_VOLTAGE);
            setBeltVoltage(-IndexerConstants.Belt.FEED_VOLTAGE);
        }).withName("IndexerFeedBackwards");
    }

    /**
     * Feeds forward until a stall is detected (fuel jammed), briefly reverses
     * to clear the jam, then repeats. Keeps fuel flowing without driver
     * intervention.
     */
    public Command smartFeed() {
        return Commands.repeatingSequence(
                feed().until(stallDetected),
                feedBackwards().withTimeout(IndexerConstants.SMART_REVERSAL_TIME)).withName("IndexerSmartFeed");
    }

    public Command stop() {
        return this.runOnce(() -> {
            indexerCommandedVoltage = 0.0;
            beltCommandedVoltage = 0.0;
            indexerMotor.stopMotor();
            beltMotor.stopMotor();
        });
    }

    public boolean isFeedingForward() {
        return indexerCommandedVoltage > 0.0 && beltCommandedVoltage > 0.0;
    }

    public boolean isFeedingBackward() {
        return indexerCommandedVoltage < 0.0 && beltCommandedVoltage < 0.0;
    }

    public double getIndexerVelocityRPM() {
        return indexerMotor.getEncoder().getVelocity();
    }

    public double getBeltVelocityRPM() {
        return beltMotor.getEncoder().getVelocity();
    }

    public double getSimulationCurrentDrawAmps() {
        return simulationCurrentDrawAmps;
    }

    @Override
    public void periodic() {
        DogLog.log(getName() + "/Current", indexerMotor.getOutputCurrent(), Amps);
        DogLog.log(getName() + "/Velocity", indexerMotor.getEncoder().getVelocity(), RPM);

        // High current limits can overheat Vortex motors; log temp for monitoring
        if (IndexerConstants.Indexer.CURRENT_LIMIT >= 60) {
            DogLog.log(getName() + "/Temperature", indexerMotor.getMotorTemperature(), Celsius);
        }

        DogLog.log(getName() + "/stallDetected", stallDetected.getAsBoolean());
        DogLog.log(getName() + "/BeltCurrent", beltMotor.getOutputCurrent(), Amps);
        DogLog.log(getName() + "/BeltVelocity", beltMotor.getEncoder().getVelocity(), RPM);
    }

    @Override
    public void simulationPeriodic() {
        if (indexerSparkSimulation == null) {
            return;
        }

        double batteryVoltage = RobotController.getBatteryVoltage();
        double dtSeconds = Constants.LOOP_TIME.in(edu.wpi.first.units.Units.Seconds);

        indexerMechanismSimulation.setInputVoltage(indexerMotor.getAppliedOutput() * batteryVoltage);
        beltMechanismSimulation.setInputVoltage(beltMotor.getAppliedOutput() * batteryVoltage);
        indexerMechanismSimulation.update(dtSeconds);
        beltMechanismSimulation.update(dtSeconds);

        indexerSparkSimulation.iterate(
                indexerMechanismSimulation.getAngularVelocityRPM(), batteryVoltage, dtSeconds);
        beltSparkSimulation.iterate(
                beltMechanismSimulation.getAngularVelocityRPM(), batteryVoltage, dtSeconds);

        simulationCurrentDrawAmps = Math.min(
                Math.abs(indexerMechanismSimulation.getCurrentDrawAmps()),
                IndexerConstants.Indexer.CURRENT_LIMIT)
                + Math.min(
                        Math.abs(beltMechanismSimulation.getCurrentDrawAmps()),
                        IndexerConstants.Belt.CURRENT_LIMIT);
        indexerSparkSimulation.setMotorCurrent(
                Math.abs(indexerMechanismSimulation.getCurrentDrawAmps()));
        beltSparkSimulation.setMotorCurrent(Math.abs(beltMechanismSimulation.getCurrentDrawAmps()));

        DogLog.log(getName() + "/Simulation/CurrentDraw", simulationCurrentDrawAmps, Amps);
    }
}
