package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
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
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<Rotation2d> angleGoalSupplier;
    private final MutAngle doglogAngle = Degrees.mutable(0.0);

    private boolean autoTrackingEnabled = false;

    /**
     * @param robotPoseSupplier A supplier to provide the robot's current pose. The
     *                          rotation value is used to ensure the turret is
     *                          facing the correct direction.
     * @param angleGoalSupplier A supplier to provide the required angle from the
     *                          field's X axis that the turret should be facing.
     */
    public TurretSubsystem(Supplier<Pose2d> robotPoseSupplier, Supplier<Rotation2d> angleGoalSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.angleGoalSupplier = angleGoalSupplier;

        motor = new TalonFX(TurretConstants.MOTOR_ID);

        absoluteEncoder = new AnalogPotentiometer(
                TurretConstants.ABSOLUTE_ENCODER_CHANNEL,
                TurretConstants.ABSOLUTE_ENCODER_FULL_RANGE,
                TurretConstants.ABSOLUTE_ENCODER_OFFSET);

        // Configure motor
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

        // 3:1 gearbox into 100:1 main gear = 300:1
        config.Feedback.SensorToMechanismRatio = 300.0;

        motor.getConfigurator().apply(config);

        motor.getPosition().setUpdateFrequency(50); // 50 Hz for position control
        motor.getVelocity().setUpdateFrequency(50);
        motor.getStatorCurrent().setUpdateFrequency(50); // 50 Hz for stator current monitoring
        motor.optimizeBusUtilization();

        if (TurretConstants.USE_ABSOLUTE_ENCODER) {
            double absoluteAngleDegrees = absoluteEncoder.get();
            double absoluteAngleRotations = Units.degreesToRotations(absoluteAngleDegrees);

            motor.setPosition(absoluteAngleRotations);

            DogLog.log((getName() + "/AbsoluteEncoderInitialized"), true);
            DogLog.log((getName() + "/InitialAbsoluteAngle"), absoluteAngleDegrees, Degrees);
            DogLog.log((getName() + "/InitialMotorPosition"), absoluteAngleRotations, Rotations);
        } else {
            DogLog.log((getName() + "/AbsoluteEncoderInitialized"), false);
        }

        // Allow tuning turret angle from DogLog
        DogLog.tunable(
                (getName() + "/AngleSetPoint"),
                0.0,
                Degrees,
                (angle) -> {
                    doglogAngle.mut_replace(angle, Degrees);
                });

        // Set default command to idle
        setDefaultCommand(
                this.runOnce(() -> {
                    motor.set(0);
                }).andThen(
                        this.idle()));

        // SysID config for characterization
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        Volts.of(4),
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
    }

    /**
     * @param angle Target angle
     */
    private void setTargetAngle(Angle angle) {
        double targetRotations = angle.in(Rotations);
        targetRotations = MathUtil.clamp(
                targetRotations,
                TurretConstants.MIN_ANGLE.in(Rotations),
                TurretConstants.MAX_ANGLE.in(Rotations));

        motor.setControl(positionRequest.withPosition(targetRotations));
    }

    /**
     * @return Current turret angle in rotations
     */
    public double getAngle() {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * @return Current turret angle
     */
    public Angle getAngleMeasure() {
        return Rotations.of(getAngle());
    }

    /**
     * @return Absolute encoder raw angle in degrees
     */
    public double getAbsoluteAngleRaw() {
        return absoluteEncoder.get();
    }

    /**
     * @return Absolute encoder angle with units
     */
    public Angle getAbsoluteAngle() {
        return Degrees.of(getAbsoluteAngleRaw());
    }

    /**
     * @return true if at target, false otherwise
     */
    public boolean isAtTarget() {
        return motor.getMotionMagicAtTarget().getValue();
    }

    /**
     * @return true if within LED tolerance of target, false otherwise
     */
    public boolean isOnTarget() {
        double targetRotations = positionRequest.Position;
        double currentRotations = motor.getPosition().getValueAsDouble();
        double targetDegrees = Units.rotationsToDegrees(targetRotations);
        double currentDegrees = Units.rotationsToDegrees(currentRotations);

        double error = Math.abs(targetDegrees - currentDegrees);
        return error <= TurretConstants.LED_TOLERANCE_DEGREES;
    }

    public void enableAutoTracking() {
        autoTrackingEnabled = true;
    }

    public void disableAutoTracking() {
        autoTrackingEnabled = false;
    }

    /**
     * 
     * @return true if auto-tracking is enabled, false otherwise
     */
    public boolean isAutoTrackingEnabled() {
        return autoTrackingEnabled;
    }

    /**
     * Command to automatically aim the turret based on the robot's position on the
     * field.
     * 
     * 1. In our alliance zone: Aim directly at the hub center
     * 2. In opponent zone, above hub: Aim at the top free space above the hub
     * 3. In opponent zone, below hub: Aim at the bottom free space below the hub
     * 
     * The turret angle is calculated as: targetFieldAngle - robotHeading
     * This converts the field-relative angle to the target into a robot-relative
     * angle.
     * 
     * @return Command that continuously updates turret angle to track the target
     */
    public Command autoTrack() {
        return this.runOnce(() -> {
            enableAutoTracking();
        }).andThen(this.run(() -> {
            // Determine the field-relative angle to aim at based on robot position
            Rotation2d targetFieldAngle = angleGoalSupplier.get();

            // Convert field-relative angle to robot-relative angle
            Rotation2d robotHeading = robotPoseSupplier.get().getRotation();
            Rotation2d turretAngle = targetFieldAngle.minus(robotHeading);

            setTargetAngle(Degrees.of(turretAngle.getDegrees()));

            DogLog.log((getName() + "/AutoTrackingActive"), true);
            DogLog.log((getName() + "/TargetFieldAngle"), targetFieldAngle.getDegrees(), Degrees);
            DogLog.log((getName() + "/RobotHeading"), robotHeading.getDegrees(), Degrees);
            DogLog.log((getName() + "/CalculatedTurretAngle"), turretAngle.getDegrees(), Degrees);
        })).finallyDo(() -> {
            disableAutoTracking();
        }).withName("TurretAutoTrack");
    }

    /**
     * @return Command to stop auto-tracking
     */
    public Command stopAutoTracking() {
        return this.run(() -> {
            motor.set(0);
        }).withName("TurretStopAutoTracking");
    }

    /**
     * @return Command to move left (increases angle counter-clockwise)
     */
    public Command moveLeft() {
        return this.run(() -> {
            motor.set(TurretConstants.MANUAL_SPEED);
        }).onlyWhile(() -> {
            // Only allow moving left if angle is below maximum
            return getAbsoluteAngle().lt(TurretConstants.MAX_ANGLE);
        }).withName("TurretMoveLeft");
    }

    /**
     * @return Command to move right (decreases angle clockwise)
     */
    public Command moveRight() {
        return this.run(() -> {
            motor.set(-TurretConstants.MANUAL_SPEED);
        }).onlyWhile(() -> {
            // Only allow moving right if angle is above minimum
            return getAbsoluteAngle().gt(TurretConstants.MIN_ANGLE);
        }).withName("TurretMoveRight");
    }

    /**
     * @param angle Target angle
     * @return Command to move to angle
     */
    public Command moveToAngle(Angle angle) {
        return this.run(() -> {
            setTargetAngle(angle);
        }).withName("TurretMoveToAngle");
    }

    /**
     * @return Command to move the turret to the current DogLog angle setpoint
     */
    public Command moveToDogLogAngle() {
        return moveToAngle(doglogAngle).withName("TurretMoveToDogLogAngle");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        DogLog.log((getName() + "/Angle"), getAngle(), Rotations);
        DogLog.log((getName() + "/AngleDegrees"), getAngleMeasure().in(Degrees), Degrees);
        DogLog.log((getName() + "/AtTarget"), isAtTarget());
        DogLog.log((getName() + "/OnTarget"), isOnTarget());
        DogLog.log((getName() + "/AutoTracking"), autoTrackingEnabled);
        DogLog.log((getName() + "/MotorOutput"), motor.get());
        DogLog.log((getName() + "/MotorCurrent"), motor.getStatorCurrent().getValue());

        if (TurretConstants.USE_ABSOLUTE_ENCODER) {
            DogLog.log((getName() + "/AbsoluteAngleDegrees"), getAbsoluteAngleRaw(), Degrees);
        }
    }
}