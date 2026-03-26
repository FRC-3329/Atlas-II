package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FlywheelConstants.Flywheel;
import frc.robot.constants.FlywheelConstants.Hood;
import frc.robot.utils.ShotParameters;

public class FlywheelSubsystem extends SubsystemBase {
    private final TalonFX left, right, hood;

    private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);

    private final Supplier<ShotParameters.Parameters> shotParametersSupplier;

    private final VoltageOut sysIdControl = new VoltageOut(0);
    private final SysIdRoutine sysIdRoutine;

    private final Trigger spikeDetected;

    private boolean varyingRPMEnabled = true;

    private final MutAngularVelocity doglogVelocity = RPM.mutable(0.0);
    private final MutAngle doglogAngle = Degrees.mutable(0.0);

    public FlywheelSubsystem(Supplier<ShotParameters.Parameters> shotParametersSupplier) {
        this.shotParametersSupplier = shotParametersSupplier;
        this.left = new TalonFX(Flywheel.LEFT_ID);
        this.right = new TalonFX(Flywheel.RIGHT_ID);
        this.hood = new TalonFX(Hood.HOOD_ID);
        this.spikeDetected = new Trigger(() -> getHoodCurrent() > Hood.ZEROING_CURRENT_THRESHOLD)
                .debounce(Hood.STALL_DEBOUNCE_TIME);

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        Slot0Configs flywheelSlot0 = flywheelConfig.Slot0;

        flywheelSlot0.kS = Flywheel.kS;
        flywheelSlot0.kV = Flywheel.kV;
        flywheelSlot0.kA = Flywheel.kA;
        flywheelSlot0.kP = Flywheel.kP;
        flywheelSlot0.kI = Flywheel.kI;
        flywheelSlot0.kD = Flywheel.kD;

        flywheelConfig.MotionMagic.MotionMagicCruiseVelocity = Flywheel.CRUISE_VELOCITY;
        flywheelConfig.MotionMagic.MotionMagicAcceleration = Flywheel.ACCELERATION;
        // flywheelConfig.MotionMagic.MotionMagicJerk = Flywheel.JERK;

        flywheelConfig.CurrentLimits.SupplyCurrentLimit = Flywheel.CURRENT_LIMIT;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        flywheelConfig.MotorOutput.Inverted = Flywheel.INVERTED;
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        left.getConfigurator().apply(flywheelConfig);

        // Right motor spins opposite direction since the wheels face each other
        flywheelConfig.MotorOutput.Inverted = (Flywheel.INVERTED == InvertedValue.CounterClockwise_Positive)
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        right.getConfigurator().apply(flywheelConfig);

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        Slot0Configs hoodSlot0 = hoodConfig.Slot0;

        hoodSlot0.kP = Hood.kP;
        hoodSlot0.kI = Hood.kI;
        hoodSlot0.kD = Hood.kD;
        hoodSlot0.kS = Hood.kS;
        hoodSlot0.kV = Hood.kV;
        hoodSlot0.kA = Hood.kA;
        hoodSlot0.kG = Hood.kG;

        hoodSlot0.GravityType = GravityTypeValue.Arm_Cosine;
        hoodSlot0.GravityArmPositionOffset = Hood.GRAVITY_ARM_POSITION_OFFSET;

        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = Hood.CRUISE_VELOCITY;
        hoodConfig.MotionMagic.MotionMagicAcceleration = Hood.ACCELERATION;

        hoodConfig.CurrentLimits.SupplyCurrentLimit = Hood.CURRENT_LIMIT;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hoodConfig.MotorOutput.Inverted = Hood.INVERTED;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 9:1 gearbox with 48:24 (2:1) belt = 18:1 total reduction
        hoodConfig.Feedback.SensorToMechanismRatio = 18.0;

        hood.getConfigurator().apply(hoodConfig);

        left.getVelocity().setUpdateFrequency(50); // 50 Hz for velocity control
        right.getVelocity().setUpdateFrequency(50);
        hood.getPosition().setUpdateFrequency(50); // 50 Hz for position control
        hood.getVelocity().setUpdateFrequency(50);
        hood.getStatorCurrent().setUpdateFrequency(50); // 50 Hz for stator current monitoring

        hood.setPosition(Degrees.zero(), 2);

        // DogLog tunables for live RPM/angle adjustment during testing
        DogLog.tunable(
                (getName() + "/RPMSetPoint"),
                750.0,
                RPM,
                (angularVelocity) -> {
                    doglogVelocity.mut_replace(angularVelocity, RPM);
                });

        DogLog.tunable(
                (getName() + "/DegreesSetPoint"),
                20.0,
                Degrees,
                (angle) -> {
                    doglogAngle.mut_replace(angle, Degrees);
                });

        setDefaultCommand(
                this.runOnce(() -> {
                    left.set(0);
                    right.set(0);
                }).andThen(
                        this.idle()));

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        Volts.of(4),
                        null,
                        (state) -> {
                            SignalLogger.writeString("state", state.toString());
                        }),

                new SysIdRoutine.Mechanism(
                        (volts) -> {
                            left.setControl(sysIdControl.withOutput(volts.in(Volts)));
                            right.setControl(sysIdControl.withOutput(volts.in(Volts)));
                        },
                        null,
                        this));

        SmartDashboard.putData("ZeroHood", zeroHood());

        DogLog.log(
                (getName() + "/VaryingRPMEnabled"),
                varyingRPMEnabled);

        left.optimizeBusUtilization();
        right.optimizeBusUtilization();
        hood.optimizeBusUtilization();
    }

    public Command setSpeed(double speed) {
        return this.run(() -> {
            left.set(speed);
            right.set(speed);
        }).withName("FlywheelSetSpeed");
    }

    /** Shoots using distance-interpolated RPM and hood angle from the shot parameter map. */
    public Command shoot() {
        return this.run(() -> {
            ShotParameters.Parameters params = shotParametersSupplier.get();
            left.setControl(request.withVelocity(params.flywheelRPS()));
            right.setControl(request.withVelocity(params.flywheelRPS()));
            hood.setControl(hoodRequest.withPosition(params.hoodRotations()));

            DogLog.log(getName() + "/AngularVelocitySetPoint", params.flywheelRPS(), RotationsPerSecond);
            DogLog.log(getName() + "/AngularSetPoint", params.hoodRotations(), Rotations);
        }).withName("FlywheelShoot");
    }

    /** Shoots at a fixed RPM and hood angle (ignores distance). */
    public Command shoot(AngularVelocity rpm, Angle angle) {
        return this.run(() -> {
            left.setControl(request.withVelocity(rpm));
            right.setControl(request.withVelocity(rpm));
            hood.setControl(hoodRequest.withPosition(angle));
        }).withName("FlywheelShootFixed");
    }

    public boolean isVaryingRPMEnabled() {
        return varyingRPMEnabled;
    }

    public void enableVaryingRPM() {
        varyingRPMEnabled = true;
        DogLog.log(getName() + "/VaryingRPMEnabled", true);
        DogLog.clearFault("VaryingRPMDisabled");
    }

    public void disableVaryingRPM() {
        varyingRPMEnabled = false;
        DogLog.log(getName() + "/VaryingRPMEnabled", false);
        DogLog.logFault("VaryingRPMDisabled", Alert.AlertType.kWarning);
    }

    public Command toggleVaryingRPM() {
        return this.runOnce(() -> {
            if (varyingRPMEnabled) {
                disableVaryingRPM();
            } else {
                enableVaryingRPM();
            }
        }).withName("FlywheelToggleVaryingRPM");
    }

    public Command stop() {
        return this.runOnce(() -> {
            left.stopMotor();
            right.stopMotor();
        });
    }

    /**
     * Shoots at the RPM/angle set via DogLog dashboard tunables.
     * Disables the hood PID on interruption so it doesn't fight gravity while idle.
     */
    public Command tunableShoot() {
        return shoot(doglogVelocity, doglogAngle)
                .finallyDo(() -> hood.set(0))
                .withName("FlywheelTunableShoot");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    /**
     * Zeros the hood position by driving it into the hard-stop and detecting
     * the resulting current spike.
     */
    public Command zeroHood() {
        return this
                .run(() -> {
                    setHoodVoltage(Hood.ZEROING_VOLTAGE);
                    DogLog.log((getName() + "/HoodStatorCurrent"), getHoodCurrent());
                })
                .until(spikeDetected)
                .finallyDo(() -> {
                    stopHood();
                    zeroHoodPosition();
                })
                .withTimeout(7.0)
                .withName("ZeroHoodFlywheelCommand");
    }

    public boolean isAtSpeed() {
        boolean leftAtSpeed = left.getMotionMagicAtTarget().getValue();
        boolean rightAtSpeed = right.getMotionMagicAtTarget().getValue();

        return leftAtSpeed && rightAtSpeed;
    }

    public double getHoodCurrent() {
        return hood.getStatorCurrent().getValueAsDouble();
    }

    public void setHoodVoltage(double voltage) {
        hood.setVoltage(voltage);
    }

    /**
     * Applies a small negative offset because the hard-stop isn't exactly at
     * the mechanical zero.
     */
    public void zeroHoodPosition() {
        hood.setPosition(Degrees.of(-2.36));
    }

    public void stopHood() {
        hood.stopMotor();
    }

    @Override
    public void periodic() {
        DogLog.log(
                (getName() + "/AtSpeed"),
                isAtSpeed());

        DogLog.log(
                (getName() + "/Speed"),
                left.get());

        DogLog.log(
                (getName() + "/Velocity"),
                left.getVelocity().getValue());

        DogLog.log(
                (getName() + "/HoodPosition"),
                hood.getPosition().getValue());

        DogLog.log(
                (getName() + "/HoodStatorCurrent"),
                getHoodCurrent());
    }
}
