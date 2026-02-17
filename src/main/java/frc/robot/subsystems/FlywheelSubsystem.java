package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.FlywheelConstants.Flywheel;
import frc.robot.constants.FlywheelConstants.Hood;
import frc.robot.commands.ZeroHoodCommand;

public class FlywheelSubsystem extends SubsystemBase {
    private final TalonFX left, right, hood;

    private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);

    private final InterpolatingDoubleTreeMap flywheelMap;
    private final InterpolatingDoubleTreeMap hoodMap;

    private final Supplier<Distance> hubDistanceSupplier;

    private final VoltageOut sysIdControl = new VoltageOut(0);
    private final SysIdRoutine sysIdRoutine;

    private final MutAngularVelocity doglogVelocity = RPM.mutable(0.0);
    private final MutAngle doglogAngle = Degrees.mutable(0.0);

    /**
     * @param hubDistanceSupplier supplier for the distance to the hub
     */
    public FlywheelSubsystem(Supplier<Distance> hubDistanceSupplier) {
        this.left = new TalonFX(Flywheel.LEFT_ID);
        this.right = new TalonFX(Flywheel.RIGHT_ID);
        this.hood = new TalonFX(Hood.HOOD_ID);

        this.hubDistanceSupplier = hubDistanceSupplier;

        // Flywheel config
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
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 1:2 belting = 0.5 (2x speedup)
        flywheelConfig.Feedback.SensorToMechanismRatio = 0.5;

        left.getConfigurator().apply(flywheelConfig);

        // Invert the right motor relative to the left
        flywheelConfig.MotorOutput.Inverted = (Flywheel.INVERTED == InvertedValue.CounterClockwise_Positive)
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        right.getConfigurator().apply(flywheelConfig);

        // Hood config
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
        // hoodConfig.MotionMagic.MotionMagicJerk = Hood.JERK;

        hoodConfig.CurrentLimits.SupplyCurrentLimit = Hood.CURRENT_LIMIT;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hoodConfig.MotorOutput.Inverted = Hood.INVERTED;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 9:1 gearbox with 48:24 (2:1) belting = 18:1 total reduction
        hoodConfig.Feedback.SensorToMechanismRatio = 18.0;

        hood.getConfigurator().apply(hoodConfig);

        left.getVelocity().setUpdateFrequency(50); // 50 Hz for velocity control
        right.getVelocity().setUpdateFrequency(50);
        hood.getPosition().setUpdateFrequency(50); // 50 Hz for position control
        hood.getVelocity().setUpdateFrequency(50);
        hood.getSupplyCurrent().setUpdateFrequency(50); // 50 Hz for current monitoring

        left.optimizeBusUtilization();
        right.optimizeBusUtilization();
        hood.optimizeBusUtilization();

        // Convert distance from hub to RPM
        flywheelMap = new InterpolatingDoubleTreeMap();
        // TODO: Fill proper values
        flywheelMap.put(1.0, 1000.0);
        flywheelMap.put(2.0, 2000.0);

        // Convert distance from hub to hood position
        hoodMap = new InterpolatingDoubleTreeMap();
        // TODO: Fill proper values
        hoodMap.put(1.0, 0.0);
        hoodMap.put(2.0, 10.0);

        // Change target RPM of motor from Doglog
        DogLog.tunable(
                (getName() + "/RPMSetPoint"),
                0.0,
                RPM,
                (angularVelocity) -> {
                    doglogVelocity.mut_replace(angularVelocity, RPM);
                });

        // Change target hood angle from Doglog
        DogLog.tunable(
                (getName() + "/DegreesSetPoint"),
                0.0,
                Degrees,
                (angle) -> {
                    doglogAngle.mut_replace(angle, Degrees);
                });

        // Set default command to idle
        setDefaultCommand(
                this.runOnce(() -> {
                    left.set(0);
                    right.set(0);
                }).andThen(
                        this.idle()));

        // SysID configuration
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
    }

    /**
     * Set the speed of motors
     * 
     * @param speed Speed of both motors
     */
    public Command setSpeed(double speed) {
        return this.run(() -> {
            left.set(speed);
            right.set(speed);
        }).withName("FlywheelSetSpeed");
    }

    /** Shoot the flywheel at the appropriate speed based on distance to the hub */
    public Command shoot() {
        return this.run(() -> {
            // Get distance to hub from supplier
            Distance dist = hubDistanceSupplier.get();

            double rps = flywheelMap.get(dist.in(Meters));
            double hoodPos = hoodMap.get(dist.in(Meters));

            left.setControl(request.withVelocity(rps));
            right.setControl(request.withVelocity(rps));
            hood.setControl(hoodRequest.withPosition(hoodPos));

            DogLog.log(
                    (getName() + "/DistanceToHub"),
                    dist);
        }).withName("FlywheelShoot");
    }

    /**
     * Shoot the flywheel at a fixed RPM and hood angle
     *
     * @param rpm   Target RPM
     * @param angle Target hood angle
     */
    public Command shoot(AngularVelocity rpm, Angle angle) {
        return this.run(() -> {
            left.setControl(request.withVelocity(rpm));
            right.setControl(request.withVelocity(rpm));
            hood.setControl(hoodRequest.withPosition(angle));
        }).withName("FlywheelShootFixed");
    }

    /**
     * Shoot the flywheel at the angular velocity and hood angle specified by
     * DogLog's angular velocity and angle setpoint. Disables the hood PID on
     * command interruption.
     * 
     * @return the command to shoot based on DogLog values
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
     * Command to zero the hood by detecting current spike
     * 
     * @return Command that zeros the hood position
     */
    public Command zeroHood() {
        return new ZeroHoodCommand(this);
    }

    /** Check if the flywheel is at the target speed */
    public boolean isAtSpeed() {
        boolean leftAtSpeed = left.getMotionMagicAtTarget().getValue();
        boolean rightAtSpeed = right.getMotionMagicAtTarget().getValue();

        return leftAtSpeed && rightAtSpeed;
    }

    /**
     * Get the hood motor's supply current
     * 
     * @return Current in amps
     */
    public double getHoodCurrent() {
        return hood.getSupplyCurrent().getValueAsDouble();
    }

    /**
     * Set the hood motor voltage directly
     * 
     * @param voltage Voltage to apply to hood motor
     */
    public void setHoodVoltage(double voltage) {
        hood.setVoltage(voltage);
    }

    /**
     * Set the hood position to zero
     */
    public void zeroHoodPosition() {
        hood.setPosition(0.0);
    }

    /**
     * Stop the hood motor
     */
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
    }
}
