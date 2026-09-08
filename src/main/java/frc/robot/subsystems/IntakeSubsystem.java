package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.sim.SparkFlexSim;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;

public class IntakeSubsystem extends SubsystemBase {
    public enum IntakeState {
        UP,
        DOWN
    }

    private final TalonFX pivotMotor;
    private final SparkFlex rollerMotor;
    private final CANcoder cancoder;

    private final TalonFXSimState pivotSimState;
    private final CANcoderSimState cancoderSimState;
    private final SingleJointedArmSim pivotSimulation;
    private final SparkFlexSim rollerSparkSimulation;
    private final DCMotorSim rollerMechanismSimulation;
    private final MechanismLigament2d pivotLigament;

    private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0);
    private final DutyCycleOut pivotDisableRequest = new DutyCycleOut(0);
    private final MutAngle doglogangle = Degrees.mutable(0.0);
    private final MutAngle pivotAngle = Rotations.mutable(0.0);
    private final MutAngularVelocity doglogVel = RPM.mutable(0.0);

    private IntakeState currentState = IntakeState.UP;
    private double targetAngleRotations = 0.0;
    private double rollerCommandedVoltage;
    private volatile double simulationCurrentDrawAmps;

    public IntakeSubsystem() {
        pivotMotor = new TalonFX(IntakeConstants.Pivot.MOTOR_ID);

        cancoder = new CANcoder(IntakeConstants.Pivot.CANCODER_ID);
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.Pivot.CANCODER_OFFSET.in(Rotations);
        cancoderConfig.MagnetSensor.SensorDirection = IntakeConstants.Pivot.CANCODER_DIRECTION;
        cancoder.getConfigurator().apply(cancoderConfig, 1);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        Slot0Configs pivotSlot0 = pivotConfig.Slot0;

        pivotSlot0.kP = IntakeConstants.Pivot.kP;
        pivotSlot0.kI = IntakeConstants.Pivot.kI;
        pivotSlot0.kD = IntakeConstants.Pivot.kD;
        pivotSlot0.kS = IntakeConstants.Pivot.kS;
        pivotSlot0.kV = IntakeConstants.Pivot.kV;
        pivotSlot0.kA = IntakeConstants.Pivot.kA;
        pivotSlot0.kG = IntakeConstants.Pivot.kG;

        pivotSlot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotSlot0.GravityArmPositionOffset = IntakeConstants.Pivot.GRAVITY_ARM_POSITION_OFFSET;

        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.Pivot.CRUISE_VELOCITY;
        pivotConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.Pivot.ACCELERATION;
        pivotConfig.MotionMagic.MotionMagicJerk = IntakeConstants.Pivot.JERK;

        pivotConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.Pivot.CURRENT_LIMIT;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotConfig.MotorOutput.Inverted = IntakeConstants.Pivot.INVERTED;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivotConfig.Feedback.SensorToMechanismRatio = 1.0;
        pivotConfig.Feedback.RotorToSensorRatio = IntakeConstants.Pivot.GEARING_RATIO;

        pivotMotor.getConfigurator().apply(pivotConfig, 1);

        pivotMotor.getPosition().setUpdateFrequency(50); // 50 Hz for position control
        pivotMotor.getVelocity().setUpdateFrequency(50);

        rollerMotor = new SparkFlex(IntakeConstants.Roller.MOTOR_ID, MotorType.kBrushless);

        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig.smartCurrentLimit(IntakeConstants.Roller.CURRENT_LIMIT);
        rollerConfig.inverted(IntakeConstants.Roller.INVERTED);
        rollerConfig.idleMode(IntakeConstants.Roller.IDLE_MODE);
        rollerConfig.encoder.quadratureMeasurementPeriod(1);
        rollerConfig.encoder.quadratureAverageDepth(3);
        rollerConfig.voltageCompensation(IntakeConstants.Roller.VOLTAGE_COMPENSATION);

        rollerConfig.signals
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

        rollerConfig.closedLoop.feedForward.kV(IntakeConstants.Roller.kV);
        rollerConfig.closedLoop.p(IntakeConstants.Roller.greg);

        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isSimulation()) {
            DCMotor pivotMotorModel = DCMotor.getKrakenX60Foc(1);
            pivotSimulation = new SingleJointedArmSim(
                    LinearSystemId.identifyPositionSystem(
                            IntakeConstants.Pivot.kV / (2.0 * Math.PI),
                            IntakeConstants.Pivot.kA / (2.0 * Math.PI)),
                    pivotMotorModel,
                    IntakeConstants.Pivot.GEARING_RATIO,
                    IntakeConstants.Pivot.LENGTH_METERS,
                    IntakeConstants.Pivot.MIN_ANGLE.in(edu.wpi.first.units.Units.Radians),
                    IntakeConstants.Pivot.MAX_ANGLE.in(edu.wpi.first.units.Units.Radians),
                    true,
                    IntakeConstants.Pivot.UP_ANGLE.in(edu.wpi.first.units.Units.Radians));
            pivotSimState = pivotMotor.getSimState();
            cancoderSimState = cancoder.getSimState();
            cancoderSimState.SensorOffset = IntakeConstants.Pivot.CANCODER_OFFSET.in(Rotations);
            setSimulatedPivotSensorState(
                    IntakeConstants.Pivot.UP_ANGLE.in(Rotations), 0.0);

            DCMotor rollerMotorModel = DCMotor.getNeoVortex(1);
            rollerSparkSimulation = new SparkFlexSim(rollerMotor, rollerMotorModel);
            rollerMechanismSimulation = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            rollerMotorModel,
                            IntakeConstants.Roller.MOMENT_OF_INERTIA,
                            1.0),
                    rollerMotorModel);

            Mechanism2d mechanism = new Mechanism2d(1.0, 1.0);
            pivotLigament = mechanism.getRoot("IntakePivot", 0.5, 0.25)
                    .append(new MechanismLigament2d(
                            "Intake",
                            IntakeConstants.Pivot.LENGTH_METERS,
                            IntakeConstants.Pivot.UP_ANGLE.in(Degrees),
                            8.0,
                            new Color8Bit(Color.kYellow)));
            SmartDashboard.putData("Simulation/Intake", mechanism);
            SimulatedBattery.addElectricalAppliances(
                    () -> edu.wpi.first.units.Units.Amps.of(simulationCurrentDrawAmps));
        } else {
            pivotSimState = null;
            cancoderSimState = null;
            pivotSimulation = null;
            rollerSparkSimulation = null;
            rollerMechanismSimulation = null;
            pivotLigament = null;
        }

        // DogLog tunable for testing specific pivot angles without redeploying
        DogLog.tunable(
                (getName() + "/PivotAngleSetPoint"),
                0.0,
                Degrees,
                (angle) -> {
                    doglogangle.mut_replace(angle, Degrees);
                });

        DogLog.tunable(
                (getName() + "/RollerVelocityRPM"),
                0.0,
                RPM,
                (vel) -> {
                    doglogVel.mut_replace(vel, RPM);
                });

        DogLog.tunable((getName() + "/kV"), 0.0, (kV) -> {
            rollerConfig.closedLoop.feedForward.kV(kV);
            rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });

        DogLog.tunable((getName() + "/kP"), 0.0, (kP) -> {
            rollerConfig.closedLoop.p(kP);
            rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });

        setDefaultCommand(
                this.runOnce(() -> {
                    rollerCommandedVoltage = 0.0;
                    rollerMotor.setVoltage(0);
                }).andThen(
                        this.idle()));

        pivotMotor.optimizeBusUtilization();
    }

    public double getPivotAngle() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public Angle getPivotAngleMeasure() {
        return pivotAngle.mut_replace(getPivotAngle(), Rotations);
    }

    private void setPivotAngle(Angle angle) {
        targetAngleRotations = angle.in(Rotations);
        pivotMotor.setControl(pivotPositionRequest.withPosition(targetAngleRotations));
        DogLog.log((getName() + "/PivotPIDEnabled"), true);
    }

    public IntakeState getState() {
        return currentState;
    }

    public boolean isDown() {
        return currentState == IntakeState.DOWN;
    }

    public boolean isPivotAtTarget() {
        double toleranceRotations = IntakeConstants.Pivot.PIVOT_TOLERANCE.in(Rotations);
        return Math.abs(getPivotAngle() - targetAngleRotations) <= toleranceRotations;
    }

    public Command tunePID() {
        return this.runOnce(
                () -> rollerMotor.getClosedLoopController().setSetpoint(doglogVel.in(RPM), ControlType.kVelocity))
                .andThen(this.idle())
                .finallyDo(() -> rollerMotor.stopMotor());
    }

    /**
     * Lowers the intake then disables PID once arrived so the motor doesn't
     * fight gravity or burn current while resting on the ground.
     */
    public Command lower() {
        return Commands.runOnce(() -> {
            setPivotAngle(IntakeConstants.Pivot.DOWN_ANGLE);
            currentState = IntakeState.DOWN;
            DogLog.log((getName() + "/IsDown"), true);
        }).andThen(
                Commands.waitUntil(this::isPivotAtTarget).withTimeout(2.0)
                        .andThen(disablePivotPIDCommand()))
                .withName("IntakeLower");
    }

    public Command kick() {
        return Commands.runOnce(() -> {
            setPivotAngle(IntakeConstants.Pivot.KICK_ANGLE);
        });
    }

    public Command raise() {
        return this.runOnce(() -> {
            setPivotAngle(IntakeConstants.Pivot.UP_ANGLE);
            currentState = IntakeState.UP;
            DogLog.log((getName() + "/IsDown"), false);
        }).withName("IntakeRaise");
    }

    /** Moves to the DogLog tunable angle; disables PID when interrupted. */
    public Command moveToDogLogAngle() {
        return this
                .runOnce(() -> setPivotAngle(doglogangle))
                .andThen(this.idle())
                .finallyDo(this::disablePivotPID)
                .withName("IntakeMoveToDogLogAngle");
    }

    /**
     * Guards against running the roller while the intake is up,
     * which would eject fuel onto the field.
     */
    public Command intakeForward() {
        return new ConditionalCommand(
                this.run(() -> {
                    //rollerMotor.getClosedLoopController().setSetpoint(IntakeConstants.Roller.INTAKE_RPM,
                    //        ControlType.kVelocity);
                    setRollerVoltage(IntakeConstants.Roller.INTAKE_VOLTAGE);
                }),
                this.runOnce(() -> {
                    DogLog.log(getName() + "/IntakeForwardBlocked",
                            "Intake forward command blocked - intake is up");
                }),
                this::isDown)
                .withName("IntakeForward");
    }

    /** Guards against running the roller backward while the intake is up. */
    public Command intakeBackward() {
        return new ConditionalCommand(
                this.run(() -> {
                    setRollerVoltage(IntakeConstants.Roller.OUTTAKE_VOLTAGE);
                }),
                this.runOnce(() -> {
                    DogLog.log(getName() + "/IntakeBackwardBlocked",
                            "Intake backward command blocked - intake is up");
                }),
                this::isDown)
                .withName("IntakeBackward");
    }

    public Command disablePivotPIDCommand() {
        return Commands.runOnce(this::disablePivotPID).withName("IntakeDisablePivotPID");
    }

    /** Sets zero duty cycle output so the motor stops holding position. */
    private void disablePivotPID() {
        pivotMotor.setControl(pivotDisableRequest);
        DogLog.log((getName() + "/PivotPIDEnabled"), false);
    }

    public Command stop() {
        return this.runOnce(() -> {
            pivotMotor.stopMotor();
            rollerMotor.stopMotor();
            rollerCommandedVoltage = 0.0;
        });
    }

    private void setRollerVoltage(double voltage) {
        rollerCommandedVoltage = voltage;
        rollerMotor.setVoltage(voltage);
    }

    public boolean isRollerRunningForward() {
        return rollerCommandedVoltage > 0.0;
    }

    public boolean isRollerRunningBackward() {
        return rollerCommandedVoltage < 0.0;
    }

    public double getRollerVelocityRPM() {
        return rollerMotor.getEncoder().getVelocity();
    }

    public double getSimulationCurrentDrawAmps() {
        return simulationCurrentDrawAmps;
    }

    private void setSimulatedPivotSensorState(double positionRotations, double velocityRps) {
        pivotSimState.setRawRotorPosition(positionRotations * IntakeConstants.Pivot.GEARING_RATIO);
        pivotSimState.setRotorVelocity(velocityRps * IntakeConstants.Pivot.GEARING_RATIO);
        cancoderSimState.setRawPosition(positionRotations);
        cancoderSimState.setVelocity(velocityRps);
        cancoder.getAbsolutePosition().refresh();
        cancoder.getPosition().refresh();
        cancoder.getVelocity().refresh();
        pivotMotor.getPosition().refresh();
        pivotMotor.getVelocity().refresh();
    }

    @Override
    public void periodic() {
        DogLog.log((getName() + "/PivotAngle"), getPivotAngle());
        DogLog.log((getName() + "/PivotAngleDegrees"), getPivotAngleMeasure().in(Degrees), Degrees);
        DogLog.log((getName() + "/PivotCurrent"), pivotMotor.getSupplyCurrent().getValueAsDouble(), Amps);
        DogLog.log((getName() + "/PivotVelocity"), pivotMotor.getVelocity().getValueAsDouble());

        DogLog.log((getName() + "/RollerVoltage"), rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage(), Volts);
        DogLog.log((getName() + "/RollerCurrent"), rollerMotor.getOutputCurrent(), Amps);
        DogLog.log((getName() + "/RollerVelocity"), rollerMotor.getEncoder().getVelocity(), RPM);
        DogLog.log((getName() + "/RollerTemp"), rollerMotor.getMotorTemperature(), Celsius);

        DogLog.log((getName() + "/State"), currentState.toString());

        if (!isDown()) {
            DogLog.logFault("Intake pivot is up", Alert.AlertType.kWarning);
        } else {
            DogLog.clearFault("Intake pivot is up");
        }
    }

    @Override
    public void simulationPeriodic() {
        if (pivotSimulation == null) {
            return;
        }

        double batteryVoltage = RobotController.getBatteryVoltage();
        double dtSeconds = Constants.LOOP_TIME.in(edu.wpi.first.units.Units.Seconds);
        pivotSimState.setSupplyVoltage(batteryVoltage);
        cancoderSimState.setSupplyVoltage(batteryVoltage);

        pivotSimulation.setInputVoltage(pivotSimState.getMotorVoltage());
        pivotSimulation.update(dtSeconds);

        double pivotPositionRotations = edu.wpi.first.math.util.Units
                .radiansToRotations(pivotSimulation.getAngleRads());
        double pivotVelocityRps = pivotSimulation.getVelocityRadPerSec() / (2.0 * Math.PI);
        setSimulatedPivotSensorState(pivotPositionRotations, pivotVelocityRps);

        rollerMechanismSimulation.setInputVoltage(rollerMotor.getAppliedOutput() * batteryVoltage);
        rollerMechanismSimulation.update(dtSeconds);
        rollerSparkSimulation.iterate(
                rollerMechanismSimulation.getAngularVelocityRPM(), batteryVoltage, dtSeconds);
        rollerSparkSimulation.setMotorCurrent(
                Math.abs(rollerMechanismSimulation.getCurrentDrawAmps()));

        simulationCurrentDrawAmps = Math.min(
                Math.abs(pivotSimulation.getCurrentDrawAmps()),
                IntakeConstants.Pivot.CURRENT_LIMIT)
                + Math.min(
                        Math.abs(rollerMechanismSimulation.getCurrentDrawAmps()),
                        IntakeConstants.Roller.CURRENT_LIMIT);
        pivotLigament.setAngle(Math.toDegrees(pivotSimulation.getAngleRads()));

        DogLog.log(getName() + "/Simulation/CurrentDraw", simulationCurrentDrawAmps, Amps);
    }
}
