package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.TurretConstants;
import frc.robot.utils.GameHelpers;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final AnalogPotentiometer absoluteEncoder;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final VoltageOut sysIdControl = new VoltageOut(0);
    private final SysIdRoutine sysIdRoutine;
    private final SwerveSubsystem swerveSubsystem;
    private final GameHelpers gameHelpers;

    private boolean autoTrackingEnabled = false;

    public TurretSubsystem(SwerveSubsystem swerveSubsystem, GameHelpers gameHelpers) {
        this.swerveSubsystem = swerveSubsystem;
        this.gameHelpers = gameHelpers;

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
                    setTargetAngle(Degrees.of(angle));
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
                TurretConstants.MIN_ANGLE,
                TurretConstants.MAX_ANGLE);

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
     * Command to automatically aim the turret based on the robot's position on the field.
     * 
     * 1. In our alliance zone: Aim directly at the hub center
     * 2. In opponent zone, above hub: Aim at the top free space above the hub
     * 3. In opponent zone, below hub: Aim at the bottom free space below the hub
     * 
     * The turret angle is calculated as: targetFieldAngle - robotHeading
     * This converts the field-relative angle to the target into a robot-relative angle.
     * 
     * @return Command that continuously updates turret angle to track the target
     */
    public Command autoTrack() {
        return this.runOnce(() -> {
            enableAutoTracking();
        }).andThen(this.run(() -> {
            // Determine the field-relative angle to aim at based on robot position
            Rotation2d targetFieldAngle;

            if (gameHelpers.isInOurZone()) {
                // In our alliance zone - aim directly at hub center
                targetFieldAngle = gameHelpers.getAngleToHub();
                DogLog.log((getName() + "/TargetZone"), "OurZone");
            } else if (gameHelpers.isAboveHub()) {
                // In opponent zone and above hub - aim at top free space
                Rotation2d hubAngle = gameHelpers.getAngleToHub();
                targetFieldAngle = hubAngle.plus(Rotation2d.fromDegrees(TurretConstants.TOP_FREE_SPACE_ANGLE_OFFSET));

                DogLog.log((getName() + "/TargetZone"), "OpponentZoneTop");
            } else {
                // In opponent zone and below hub - aim at bottom free space
                Rotation2d hubAngle = gameHelpers.getAngleToHub();
                targetFieldAngle = hubAngle
                        .plus(Rotation2d.fromDegrees(TurretConstants.BOTTOM_FREE_SPACE_ANGLE_OFFSET));

                DogLog.log((getName() + "/TargetZone"), "OpponentZoneBottom");
            }

            // Convert field-relative angle to robot-relative angle
            Rotation2d robotHeading = swerveSubsystem.getGyro();
            Rotation2d turretAngle = targetFieldAngle.minus(robotHeading);
            
            // Normalize angle to [-180, 180] degrees to prevent wraparound issues
            double turretAngleDegrees = MathUtil.inputModulus(turretAngle.getDegrees(), -180.0, 180.0);

            setTargetAngle(Degrees.of(turretAngleDegrees));

            DogLog.log((getName() + "/AutoTrackingActive"), true);
            DogLog.log((getName() + "/TargetFieldAngle"), targetFieldAngle.getDegrees(), Degrees);
            DogLog.log((getName() + "/RobotHeading"), robotHeading.getDegrees(), Degrees);
            DogLog.log((getName() + "/CalculatedTurretAngle"), turretAngleDegrees, Degrees);
            DogLog.log((getName() + "/InOurZone"), gameHelpers.isInOurZone());
            DogLog.log((getName() + "/AboveHub"), gameHelpers.isAboveHub());
        })).withName("TurretAutoTrack");
    }

    /**
     * @return Command to stop auto-tracking
     */
    public Command stopAutoTracking() {
        return this.runOnce(() -> {
            disableAutoTracking();
            motor.set(0);
        }).withName("TurretStopAutoTracking");
    }

    /**
     * @return Command to move left
     */
    public Command moveLeft() {
        return this.run(() -> {
            motor.set(-TurretConstants.MANUAL_SPEED);
        }).withName("TurretMoveLeft");
    }

    /**
     * @return Command to move right
     */
    public Command moveRight() {
        return this.run(() -> {
            motor.set(TurretConstants.MANUAL_SPEED);
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
        DogLog.log((getName() + "/AutoTracking"), autoTrackingEnabled);
        DogLog.log((getName() + "/MotorOutput"), motor.get());
        DogLog.log((getName() + "/MotorCurrent"), motor.getSupplyCurrent().getValue());

        if (TurretConstants.USE_ABSOLUTE_ENCODER) {
            DogLog.log((getName() + "/AbsoluteAngleDegrees"), getAbsoluteAngleRaw(), Degrees);
        }
    }
}