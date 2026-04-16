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
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final AnalogPotentiometer absoluteEncoder;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final VoltageOut sysIdControl = new VoltageOut(0);
    private final SysIdRoutine sysIdRoutine;
    private final Supplier<Pose2d> turretPoseSupplier;
    private final Supplier<Rotation2d> angleGoalSupplier;
    private final MutAngle loggedAngle = Degrees.mutable(0.0);
    private final LoggedNetworkNumber angleSetpointDegrees;
    private final Alert autoTrackingDisabledAlert =
            new Alert("Turret auto-tracking disabled", Alert.AlertType.kWarning);

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

        // 3:1 planetary gearbox into 100:10 spur gear = 30:1 total reduction
        config.Feedback.SensorToMechanismRatio = 30.0;

        motor.getConfigurator().apply(config, 0.2);

        motor.getPosition().setUpdateFrequency(50); // 50 Hz for position control
        motor.getVelocity().setUpdateFrequency(50);
        motor.getStatorCurrent().setUpdateFrequency(50); // 50 Hz for stator current monitoring

        if (TurretConstants.USE_ABSOLUTE_ENCODER) {
            double absoluteAngleDegrees = absoluteEncoder.get();
            double absoluteAngleRotations = Units.degreesToRotations(absoluteAngleDegrees);

            motor.setPosition(absoluteAngleRotations, 1);

            Logger.recordOutput((getName() + "/AbsoluteEncoderInitialized"), true);
            Logger.recordOutput((getName() + "/InitialAbsoluteAngle"), absoluteAngleDegrees, Degrees);
            Logger.recordOutput((getName() + "/InitialMotorPosition"), absoluteAngleRotations, Rotations);
        } else {
            Logger.recordOutput((getName() + "/AbsoluteEncoderInitialized"), false);
        }

        angleSetpointDegrees = new LoggedNetworkNumber(getName() + "/AngleSetPoint", 0.0);

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

        SmartDashboard.putData("MoveToLoggedAngle", moveToLoggedAngle());
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
        Logger.recordOutput((getName() + "/AutoTracking"), true);
    }

    public void disableAutoTracking() {
        autoTrackingEnabled = false;
        Logger.recordOutput((getName() + "/AutoTracking"), false);
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

            Logger.recordOutput((getName() + "/AutoTrackingActive"), true);
            Logger.recordOutput((getName() + "/TargetFieldAngle"), targetFieldAngle.getDegrees(), Degrees);
            Logger.recordOutput((getName() + "/RobotHeading"), turretFieldSetpoint.getDegrees(), Degrees);
            Logger.recordOutput((getName() + "/CalculatedTurretAngle"), turretRobotSetpoint.getDegrees(), Degrees);
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

    public Command moveToLoggedAngle() {
        return this.run(() -> setTargetAngle(loggedAngle.mut_replace(angleSetpointDegrees.get(), Degrees)))
                .finallyDo(() -> motor.stopMotor())
                .withName("TurretMoveToLoggedAngle");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        // Periodically re-sync the motor encoder with the absolute encoder
        // to correct any drift from skipped counts or brownouts.
        if (absoluteCount++ > 100) {
            motor.setPosition(getAbsoluteAngle());
            absoluteCount = 0;
        }

        Logger.recordOutput((getName() + "/Angle"), getAngle(), Rotations);
        Logger.recordOutput((getName() + "/AngleDegrees"), getAngleMeasure().in(Degrees), Degrees);

        if (!autoTrackingEnabled) {
            autoTrackingDisabledAlert.set(true);
        } else {
            autoTrackingDisabledAlert.set(false);
        }

        Logger.recordOutput((getName() + "/MotorOutput"), motor.get());
        Logger.recordOutput((getName() + "/MotorCurrent"), motor.getStatorCurrent().getValue());

        if (TurretConstants.USE_ABSOLUTE_ENCODER) {
            Logger.recordOutput((getName() + "/AbsoluteAngleDegrees"), getAbsoluteAngleRaw(), Degrees);
        }
    }
}
