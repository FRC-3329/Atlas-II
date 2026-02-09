package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    public enum IntakeState {
        UP,
        DOWN
    }

    private final TalonFX pivotMotor;
    private final SparkMax rollerMotor;
    private final DutyCycleEncoder absoluteEncoder;

    private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0);
    private final DutyCycleOut pivotDisableRequest = new DutyCycleOut(0);

    private IntakeState currentState = IntakeState.UP;
    private boolean pivotPIDEnabled = false;

    public IntakeSubsystem() {
        pivotMotor = new TalonFX(IntakeConstants.Pivot.MOTOR_ID);

        // Configure pivot motor
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

        // 80:1 gearbox into 43:24 belting = 143.333:1
        pivotConfig.Feedback.SensorToMechanismRatio = 430.0 / 3.0;

        pivotMotor.getConfigurator().apply(pivotConfig);

        absoluteEncoder = new DutyCycleEncoder(IntakeConstants.Pivot.ABSOLUTE_ENCODER_PORT);

        Angle absoluteAngle = getAbsoluteAngle();
        pivotMotor.setPosition(absoluteAngle.in(Rotations));

        rollerMotor = new SparkMax(IntakeConstants.Roller.MOTOR_ID, MotorType.kBrushless);

        // Configure roller motor
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.smartCurrentLimit(IntakeConstants.Roller.CURRENT_LIMIT);
        rollerConfig.inverted(IntakeConstants.Roller.INVERTED);
        rollerConfig.idleMode(IntakeConstants.Roller.IDLE_MODE);
        rollerConfig.voltageCompensation(IntakeConstants.Roller.VOLTAGE_COMPENSATION);

        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Allow tuning pivot angle from DogLog
        DogLog.tunable(
                (getName() + "/PivotAngleSetPoint"),
                0.0,
                Degrees,
                (angle) -> {
                    setPivotAngle(Degrees.of(angle));
                });

        setDefaultCommand(
                this.runOnce(() -> {
                    rollerMotor.setVoltage(0);
                }).andThen(
                        this.idle()));
    }

    /**
     * @return Absolute encoder angle with offset applied
     */
    private Angle getAbsoluteAngle() {
        double encoderRotations = absoluteEncoder.get();
        Angle angle = Rotations.of(encoderRotations).plus(IntakeConstants.Pivot.ABSOLUTE_ENCODER_OFFSET);

        return angle;
    }

    /**
     * @return Current pivot angle in rotations
     */
    public double getPivotAngle() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    /**
     * @return Current pivot angle
     */
    public Angle getPivotAngleMeasure() {
        return Rotations.of(getPivotAngle());
    }

    /**
     * @param angle Target angle for pivot
     */
    private void setPivotAngle(Angle angle) {
        pivotMotor.setControl(pivotPositionRequest.withPosition(angle.in(Rotations)));
        pivotPIDEnabled = true;
    }

    /**
     * @return Current intake state
     */
    public IntakeState getState() {
        return currentState;
    }

    /**
     * @return true if intake is down, false otherwise
     */
    public boolean isDown() {
        return currentState == IntakeState.DOWN;
    }

    /**
     * @return true if pivot is at target, false otherwise
     */
    public boolean isPivotAtTarget() {
        return pivotMotor.getMotionMagicAtTarget().getValue();
    }

    /**
     * Command to lower the intake
     * Sets the pivot to down position and disables PID once down
     * 
     * @return Command to lower intake
     */
    public Command lower() {
        return this.runOnce(() -> {
            setPivotAngle(IntakeConstants.Pivot.DOWN_ANGLE);
            currentState = IntakeState.DOWN;
        }).andThen(
                // Wait until at target, then disable PID control
                Commands.waitUntil(this::isPivotAtTarget).andThen(
                        disablePivotPID()))
                .withName("IntakeLower");
    }

    /**
     * Command to raise the intake
     * Sets the pivot to up position
     * 
     * @return Command to raise intake
     */
    public Command raise() {
        return this.runOnce(() -> {
            setPivotAngle(IntakeConstants.Pivot.UP_ANGLE);
            currentState = IntakeState.UP;
        }).withName("IntakeRaise");
    }

    /**
     * Command to run the intake roller forward
     * Only runs if intake is down
     * 
     * @return Command to run intake forward
     */
    public Command intakeForward() {
        return new ConditionalCommand(
                this.run(() -> {
                    rollerMotor.setVoltage(IntakeConstants.Roller.INTAKE_VOLTAGE);
                }),
                this.runOnce(() -> {
                    DogLog.log(getName() + "/IntakeForwardBlocked",
                            "Intake forward command blocked - intake is up");
                }),
                this::isDown)
                .withName("IntakeForward");
    }

    /**
     * Command to run the intake roller backward
     * Only runs if intake is down
     * 
     * @return Command to run intake backward
     */
    public Command intakeBackward() {
        return new ConditionalCommand(
                this.run(() -> {
                    rollerMotor.setVoltage(IntakeConstants.Roller.OUTTAKE_VOLTAGE);
                }),
                this.runOnce(() -> {
                    DogLog.log(getName() + "/IntakeBackwardBlocked",
                            "Intake backward command blocked - intake is up");
                }),
                this::isDown)
                .withName("IntakeBackward");
    }

    /**
     * Command to disable pivot PID control
     * Sets zero percent duty cycle
     * 
     * @return Command to disable pivot PID
     */
    public Command disablePivotPID() {
        return this.runOnce(() -> {
            pivotMotor.setControl(pivotDisableRequest);
            pivotPIDEnabled = false;
        }).withName("IntakeDisablePivotPID");
    }

    @Override
    public void periodic() {
        DogLog.log((getName() + "/PivotAngle"), getPivotAngle());
        DogLog.log((getName() + "/PivotAngleDegrees"), getPivotAngleMeasure().in(Degrees), Degrees);
        DogLog.log((getName() + "/PivotAtTarget"), isPivotAtTarget());
        DogLog.log((getName() + "/PivotCurrent"), pivotMotor.getSupplyCurrent().getValueAsDouble(), Amps);
        DogLog.log((getName() + "/PivotVelocity"), pivotMotor.getVelocity().getValueAsDouble());

        DogLog.log((getName() + "/AbsoluteAngleDegrees"), getAbsoluteAngle().in(Degrees), Degrees);

        DogLog.log((getName() + "/RollerVoltage"), rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage(), Volts);
        DogLog.log((getName() + "/RollerCurrent"), rollerMotor.getOutputCurrent(), Amps);
        DogLog.log((getName() + "/RollerVelocity"), rollerMotor.getEncoder().getVelocity(), RPM);

        DogLog.log((getName() + "/State"), currentState.toString());
        DogLog.log((getName() + "/IsDown"), isDown());
        DogLog.log((getName() + "/PivotPIDEnabled"), pivotPIDEnabled);
    }
}
