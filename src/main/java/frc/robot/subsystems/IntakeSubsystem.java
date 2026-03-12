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
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final MutAngle doglogangle = Degrees.mutable(0.0);

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

        pivotConfig.Feedback.SensorToMechanismRatio = 4.0 * 5.0 * 43.0 / 24.0;

        pivotMotor.getConfigurator().apply(pivotConfig, 1);

        pivotMotor.getPosition().setUpdateFrequency(50); // 50 Hz for position control
        pivotMotor.getVelocity().setUpdateFrequency(50);
        pivotMotor.optimizeBusUtilization();

        absoluteEncoder = new DutyCycleEncoder(IntakeConstants.Pivot.ABSOLUTE_ENCODER_PORT);
        rollerMotor = new SparkMax(IntakeConstants.Roller.MOTOR_ID, MotorType.kBrushless);

        // Configure roller motor
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.smartCurrentLimit(IntakeConstants.Roller.CURRENT_LIMIT);
        rollerConfig.inverted(IntakeConstants.Roller.INVERTED);
        rollerConfig.idleMode(IntakeConstants.Roller.IDLE_MODE);
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

        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Allow tuning pivot angle from DogLog
        DogLog.tunable(
                (getName() + "/PivotAngleSetPoint"),
                0.0,
                Degrees,
                (angle) -> {
                    doglogangle.mut_replace(angle, Degrees);
                });

        setDefaultCommand(
                this.runOnce(() -> {
                    rollerMotor.setVoltage(0);
                }).andThen(
                        this.idle()));

        pivotMotor.setPosition(IntakeConstants.Pivot.STARTING_ANGLE);

        SmartDashboard.putData("SetPivotUpStartingAngle", setPivotStartingCommand());
        SmartDashboard.putData("SetPivotDownAngle", setPivotDownAngleCommand());
    }

    public Command setPivotStartingCommand() {
        return this.runOnce(() -> {
            currentState = IntakeState.UP;
            pivotMotor.setPosition(IntakeConstants.Pivot.STARTING_ANGLE);
        })
                .ignoringDisable(true)
                .withName("Set Pivot Angle Up");
    }

    public Command setPivotDownAngleCommand() {
        return this.runOnce(() -> {
            currentState = IntakeState.DOWN;
            pivotMotor.setPosition(IntakeConstants.Pivot.DOWN_ANGLE);
        })
                .ignoringDisable(true)
                .withName("Set Pivot Angle Down");
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
                Commands.waitUntil(this::isPivotAtTarget).withTimeout(2.0)
                        .andThen(disablePivotPIDCommand()))
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
     * @return Command to move to the current DogLog angle. Disables PID when
     *         interrupted.
     */
    public Command moveToDogLogAngle() {
        return this
                .runOnce(() -> setPivotAngle(doglogangle))
                .andThen(this.idle())
                .finallyDo(this::disablePivotPID)
                .withName("IntakeMoveToDogLogAngle");
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
    public Command disablePivotPIDCommand() {
        return this.runOnce(this::disablePivotPID).withName("IntakeDisablePivotPID");
    }

    /**
     * Disable's pivot PID control by setting zero percent duty cycle
     */
    private void disablePivotPID() {
        pivotMotor.setControl(pivotDisableRequest);
        pivotPIDEnabled = false;
    }

    public Command stop() {
        return this.runOnce(() -> {
            pivotMotor.stopMotor();
            rollerMotor.stopMotor();
        });
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

        if (!isDown()) {
            DogLog.logFault("Intake pivot is up", Alert.AlertType.kWarning);
        } else {
            DogLog.clearFault("Intake pivot is up");
        }
    }
}
