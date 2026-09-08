package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final AnalogPotentiometer absoluteEncoder;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final VoltageOut sysIdControl = new VoltageOut(0);
    private final SysIdRoutine sysIdRoutine;
    private final Supplier<Pose2d> turretPoseSupplier;
    private final Supplier<Rotation2d> angleGoalSupplier;
    private final MutAngle doglogAngle = Degrees.mutable(0.0);

    private final AnalogInputSim absoluteEncoderSimulation;
    private final TalonFXSimState motorSimState;
    private final SingleJointedArmSim turretSimulation;
    private final MechanismLigament2d turretLigament;
    private volatile double simulationCurrentDrawAmps;

    private boolean autoTrackingEnabled = false;
    private double trueTargetRotations = 0.0;
    private int absoluteCount = 0;

    /**
     * @param robotPoseSupplier A supplier to provide the robot's current pose. The
     *                          rotation value is used to ensure the turret is
     *                          facing the correct direction.
     * @param angleGoalSupplier A supplier to provide the required angle from the
     *                          field's X axis that the turret should be facing.
     */
    public TurretSubsystem(Supplier<Pose2d> robotPoseSupplier, Supplier<Rotation2d> angleGoalSupplier) {
        this.turretPoseSupplier = robotPoseSupplier;
        this.angleGoalSupplier = angleGoalSupplier;

        absoluteEncoder = new AnalogPotentiometer(
                TurretConstants.ABSOLUTE_ENCODER_CHANNEL,
                TurretConstants.ABSOLUTE_ENCODER_FULL_RANGE,
                TurretConstants.ABSOLUTE_ENCODER_OFFSET);
        if (RobotBase.isSimulation()) {
            absoluteEncoderSimulation = new AnalogInputSim(TurretConstants.ABSOLUTE_ENCODER_CHANNEL);
            setSimulatedAbsoluteAngle(0.0);
        } else {
            absoluteEncoderSimulation = null;
        }

        motor = new TalonFX(TurretConstants.MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        Slot0Configs slot0 = config.Slot0;

        slot0.kP = TurretConstants.kP;
        slot0.kI = TurretConstants.kI;
        slot0.kD = TurretConstants.kD;
        slot0.kS = TurretConstants.kS;
        slot0.kV = TurretConstants.kV;
        slot0.kA = TurretConstants.kA;

        config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = TurretConstants.ACCELERATION;
        config.MotionMagic.MotionMagicJerk = TurretConstants.JERK;

        config.CurrentLimits.SupplyCurrentLimit = TurretConstants.CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = TurretConstants.INVERTED;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Feedback.SensorToMechanismRatio = TurretConstants.GEARING_RATIO;

        motor.getConfigurator().apply(config, 0.2);

        if (RobotBase.isSimulation()) {
            DCMotor motorModel = DCMotor.getKrakenX60Foc(1);
            turretSimulation = new SingleJointedArmSim(
                    LinearSystemId.identifyPositionSystem(
                            TurretConstants.kV / (2.0 * Math.PI),
                            TurretConstants.kA / (2.0 * Math.PI)),
                    motorModel,
                    TurretConstants.GEARING_RATIO,
                    TurretConstants.LENGTH_METERS,
                    TurretConstants.MIN_ANGLE.in(edu.wpi.first.units.Units.Radians),
                    TurretConstants.MAX_ANGLE.in(edu.wpi.first.units.Units.Radians),
                    false,
                    0.0);
            motorSimState = motor.getSimState();
            motorSimState.Orientation = ChassisReference.Clockwise_Positive;
            setSimulatedMotorSensorState(0.0, 0.0);

            Mechanism2d mechanism = new Mechanism2d(1.0, 1.0);
            turretLigament = mechanism.getRoot("TurretPivot", 0.5, 0.5)
                    .append(new MechanismLigament2d(
                            "Turret",
                            TurretConstants.LENGTH_METERS,
                            0.0,
                            8.0,
                            new Color8Bit(Color.kRed)));
            SmartDashboard.putData("Simulation/Turret", mechanism);
            SimulatedBattery.addElectricalAppliances(
                    () -> edu.wpi.first.units.Units.Amps.of(simulationCurrentDrawAmps));
        } else {
            turretSimulation = null;
            motorSimState = null;
            turretLigament = null;
        }

        motor.getPosition().setUpdateFrequency(50); // 50 Hz for position control
        motor.getVelocity().setUpdateFrequency(50);
        motor.getStatorCurrent().setUpdateFrequency(50); // 50 Hz for stator current monitoring

        if (TurretConstants.USE_ABSOLUTE_ENCODER) {
            double absoluteAngleDegrees = absoluteEncoder.get();
            double absoluteAngleRotations = Units.degreesToRotations(absoluteAngleDegrees);

            motor.setPosition(absoluteAngleRotations, 1);

            DogLog.log((getName() + "/AbsoluteEncoderInitialized"), true);
            DogLog.log((getName() + "/InitialAbsoluteAngle"), absoluteAngleDegrees, Degrees);
            DogLog.log((getName() + "/InitialMotorPosition"), absoluteAngleRotations, Rotations);
        } else {
            DogLog.log((getName() + "/AbsoluteEncoderInitialized"), false);
        }

        // DogLog tunable for testing angles without redeploying
        DogLog.tunable(
                (getName() + "/AngleSetPoint"),
                0.0,
                Degrees,
                (angle) -> {
                    doglogAngle.mut_replace(angle, Degrees);
                });

        setDefaultCommand(
                this.runOnce(() -> {
                    motor.set(0);
                }).andThen(
                        this.idle()));

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.5).per(Second),
                        Volts.of(1.5),
                        null,
                        (state) -> {
                            SignalLogger.writeString("turret-state", state.toString());
                        }),
                new SysIdRoutine.Mechanism(
                        (volts) -> {
                            motor.setControl(sysIdControl.withOutput(volts.in(Volts)));
                        },
                        null,
                        this));

        SmartDashboard.putData("MoveToDogLog", moveToDogLogAngle());
        motor.optimizeBusUtilization();
    }

    private void setTargetAngle(Angle angle) {
        trueTargetRotations = angle.in(Rotations);
        double targetRotations = MathUtil.clamp(
                trueTargetRotations,
                TurretConstants.MIN_ANGLE.in(Rotations),
                TurretConstants.MAX_ANGLE.in(Rotations));

        motor.setControl(positionRequest.withPosition(targetRotations));
    }

    public double getAngle() {
        return motor.getPosition().getValueAsDouble();
    }

    public Angle getAngleMeasure() {
        return Rotations.of(getAngle());
    }

    public double getAbsoluteAngleRaw() {
        return absoluteEncoder.get();
    }

    public Angle getAbsoluteAngle() {
        return Degrees.of(getAbsoluteAngleRaw());
    }

    public boolean isAtTarget() {
        return motor.getMotionMagicAtTarget().getValue();
    }

    public double getSimulationCurrentDrawAmps() {
        return simulationCurrentDrawAmps;
    }

    /**
     * Uses a wider tolerance than MotionMagic so the LED feedback
     * activates slightly before the turret fully settles.
     */
    public boolean isOnTarget() {
        double currentRotations = motor.getPosition().getValueAsDouble();
        double currentDegrees = Units.rotationsToDegrees(currentRotations);

        double error = Math.abs(Units.rotationsToDegrees(trueTargetRotations) - currentDegrees);
        return error <= TurretConstants.LED_TOLERANCE_DEGREES;
    }

    public void enableAutoTracking() {
        autoTrackingEnabled = true;
        DogLog.log((getName() + "/AutoTracking"), true);
    }

    public void disableAutoTracking() {
        autoTrackingEnabled = false;
        DogLog.log((getName() + "/AutoTracking"), false);
    }

    public boolean isAutoTrackingEnabled() {
        return autoTrackingEnabled;
    }

    /**
     * Continuously aims the turret at the target computed by {@code GameHelpers}.
     * <p>
     * Converts the field-relative target angle to a robot-relative turret
     * angle by subtracting the robot's heading (the turret sits on the
     * robot, so field angles must become relative to the chassis).
     */
    public Command autoTrack() {
        return this.runOnce(() -> {
            enableAutoTracking();
        }).andThen(this.run(() -> {
            Rotation2d targetFieldAngle = angleGoalSupplier.get();

            // Turret is rear-mounted, so offset heading by 180°
            Rotation2d turretFieldSetpoint = turretPoseSupplier.get().getRotation().plus(Rotation2d.k180deg);
            Rotation2d turretRobotSetpoint = targetFieldAngle.minus(turretFieldSetpoint);

            setTargetAngle(Degrees.of(turretRobotSetpoint.getDegrees()));

            DogLog.log((getName() + "/AutoTrackingActive"), true);
            DogLog.log((getName() + "/TargetFieldAngle"), targetFieldAngle.getDegrees(), Degrees);
            DogLog.log((getName() + "/RobotHeading"), turretFieldSetpoint.getDegrees(), Degrees);
            DogLog.log((getName() + "/CalculatedTurretAngle"), turretRobotSetpoint.getDegrees(), Degrees);
        })).finallyDo(() -> {
            disableAutoTracking();
        }).withName("TurretAutoTrack");
    }

    public Command stopAutoTracking() {
        return this.runOnce(() -> {
            motor.set(0);
        }).withName("TurretStopAutoTracking");
    }

    public Command moveLeft() {
        return this.run(() -> {
            motor.set(TurretConstants.MANUAL_SPEED);
        }).onlyWhile(() -> {
            // Prevent exceeding soft limits to protect cable wrap
            return getAbsoluteAngle().lt(TurretConstants.MAX_ANGLE);
        }).withName("TurretMoveLeft");
    }

    public Command moveRight() {
        return this.run(() -> {
            motor.set(-TurretConstants.MANUAL_SPEED);
        }).onlyWhile(() -> {
            return getAbsoluteAngle().gt(TurretConstants.MIN_ANGLE);
        }).withName("TurretMoveRight");
    }

    public Command moveToAngle(Angle angle) {
        return this.run(() -> {
            setTargetAngle(angle);
        }).withName("TurretMoveToAngle");
    }

    public Command moveToDogLogAngle() {
        return moveToAngle(doglogAngle).finallyDo(() -> motor.stopMotor())
                .withName("TurretMoveToDogLogAngle");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    private void setSimulatedAbsoluteAngle(double angleDegrees) {
        double voltage = (angleDegrees - TurretConstants.ABSOLUTE_ENCODER_OFFSET)
                / TurretConstants.ABSOLUTE_ENCODER_FULL_RANGE * 5.0;
        absoluteEncoderSimulation.setVoltage(MathUtil.clamp(voltage, 0.0, 5.0));
    }

    private void setSimulatedMotorSensorState(double positionRotations, double velocityRps) {
        motorSimState.setRawRotorPosition(positionRotations * TurretConstants.GEARING_RATIO);
        motorSimState.setRotorVelocity(velocityRps * TurretConstants.GEARING_RATIO);
        motor.getPosition().refresh();
        motor.getVelocity().refresh();
    }

    @Override
    public void periodic() {
        // Periodically re-sync the motor encoder with the absolute encoder
        // to correct any drift from skipped counts or brownouts.
        // Simulated rotor position comes directly from the same physics state as the
        // absolute sensor, so hardware drift correction is neither needed nor useful.
        if (!RobotBase.isSimulation() && absoluteCount++ > 100) {
            motor.setPosition(getAbsoluteAngle());
            absoluteCount = 0;
        }

        DogLog.log((getName() + "/Angle"), getAngle(), Rotations);
        DogLog.log((getName() + "/AngleDegrees"), getAngleMeasure().in(Degrees), Degrees);

        if (!autoTrackingEnabled) {
            DogLog.logFault("Turret auto-tracking disabled", Alert.AlertType.kWarning);
        } else {
            DogLog.clearFault("Turret auto-tracking disabled");
        }

        DogLog.log((getName() + "/MotorOutput"), motor.get());
        DogLog.log((getName() + "/MotorCurrent"), motor.getStatorCurrent().getValue());

        if (TurretConstants.USE_ABSOLUTE_ENCODER) {
            DogLog.forceNt.log((getName() + "/AbsoluteAngleDegrees"), getAbsoluteAngleRaw(), Degrees);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (turretSimulation == null) {
            return;
        }

        double batteryVoltage = RobotController.getBatteryVoltage();
        int subTicks = SimulatedArena.getSimulationSubTicksIn1Period();
        double dtSeconds = Constants.LOOP_TIME.in(edu.wpi.first.units.Units.Seconds) / subTicks;
        motorSimState.setSupplyVoltage(batteryVoltage);

        for (int i = 0; i < subTicks; i++) {
            turretSimulation.setInputVoltage(motorSimState.getMotorVoltage());
            turretSimulation.update(dtSeconds);

            double positionRotations = edu.wpi.first.math.util.Units
                    .radiansToRotations(turretSimulation.getAngleRads());
            double velocityRps = turretSimulation.getVelocityRadPerSec() / (2.0 * Math.PI);
            setSimulatedMotorSensorState(positionRotations, velocityRps);
            setSimulatedAbsoluteAngle(Math.toDegrees(turretSimulation.getAngleRads()));
        }

        simulationCurrentDrawAmps = Math.min(
                Math.abs(turretSimulation.getCurrentDrawAmps()),
                TurretConstants.CURRENT_LIMIT);
        turretLigament.setAngle(Math.toDegrees(turretSimulation.getAngleRads()));

        DogLog.log(getName() + "/Simulation/CurrentDraw", simulationCurrentDrawAmps,
                edu.wpi.first.units.Units.Amps);
    }
}
