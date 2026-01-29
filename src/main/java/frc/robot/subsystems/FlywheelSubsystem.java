package frc.robot.subsystems;

import frc.robot.Constants;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FlywheelConstants;

public class FlywheelSubsystem extends SubsystemBase {
    private final TalonFX left, right;
    private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
    private final InterpolatingDoubleTreeMap map;
    private final Supplier<Pose2d> robotPoseSupplier;

    private final VoltageOut sysIdControl = new VoltageOut(0);
    private final SysIdRoutine sysIdRoutine;

    /**
     * @param robotPoseSupplier supplier for the robot pose2d
     */
    public FlywheelSubsystem(Supplier<Pose2d> robotPoseSupplier) {
        this.left = new TalonFX(FlywheelConstants.LEFT_ID);
        this.right = new TalonFX(FlywheelConstants.RIGHT_ID);
        this.robotPoseSupplier = robotPoseSupplier;

        TalonFXConfiguration tfxConfig = new TalonFXConfiguration();
        Slot0Configs slot0cfg = tfxConfig.Slot0;

        slot0cfg.kS = FlywheelConstants.kS;
        slot0cfg.kV = FlywheelConstants.kV;
        slot0cfg.kA = FlywheelConstants.kA;
        slot0cfg.kP = FlywheelConstants.kP;
        slot0cfg.kI = FlywheelConstants.kI;
        slot0cfg.kD = FlywheelConstants.kD;

        tfxConfig.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.CURRENT_LIMIT;
        tfxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        tfxConfig.MotorOutput.Inverted = FlywheelConstants.INVERTED;

        left.getConfigurator().apply(tfxConfig);

        // Invert the right motor relative to the left
        tfxConfig.MotorOutput.Inverted = (FlywheelConstants.INVERTED == InvertedValue.CounterClockwise_Positive)
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        right.getConfigurator().apply(tfxConfig);

        // Convert distance from hub to RPM
        map = new InterpolatingDoubleTreeMap();
        // TODO: Fill proper values
        map.put(1.0, 1000.0);
        map.put(2.0, 2000.0);

        // Change target RPM of motor from Doglog
        DogLog.tunable(
            (getName() + "/RPMSetPoint"),
            0.0,
            RPM,
            (rpm) -> {
                left.setControl(request.withVelocity(rpm));
                right.setControl(request.withVelocity(rpm));
            });

        // Set default command to idle
        setDefaultCommand(
            this.runOnce(() -> {
                left.set(0);
                right.set(0);
            }).andThen(
                this.idle()
            )
        );

        // SysID configuration
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> {
                    SignalLogger.writeString("state", state.toString());
                }
            ),

            new SysIdRoutine.Mechanism(
                (volts) -> {
                    left.setControl(sysIdControl.withOutput(volts.in(Volts)));
                    right.setControl(sysIdControl.withOutput(volts.in(Volts)));
                },
                null,
                this
            )
        );
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
        });
    }

    /** Shoot the flywheel at the appropriate speed based on distance to the hub */
    public Command shoot() {
        return this.run(() -> {
            // Calculate distance to hub
            double dist = new Transform2d(
                robotPoseSupplier.get(),
                Constants.HUB_LOCATION
            ).getTranslation().getNorm();

            double rpm = map.get(dist);

            left.setControl(request.withVelocity(rpm));
            right.setControl(request.withVelocity(rpm));
        });
    }

    /**
     * Shoot the flywheel at a fixed RPM
     *
     * @param rpm Target RPM
     */
    public Command shoot(double rpm) {
        return this.run(() -> {
            left.setControl(request.withVelocity(rpm));
            right.setControl(request.withVelocity(rpm));
        });
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    /** Check if the flywheel is at the target speed */
    public boolean isAtSpeed() {
        boolean leftAtSpeed = left.getMotionMagicAtTarget().getValue();
        boolean rightAtSpeed = right.getMotionMagicAtTarget().getValue();

        return leftAtSpeed && rightAtSpeed;
    }

    @Override
    public void periodic() {
        DogLog.log((getName() + "/AtSpeed"), isAtSpeed());
        DogLog.log((getName() + "/Speed"), left.get());
        DogLog.log(
            (getName() + "/RPM"),
            left.getVelocity()
                .getValue()
                .in(RPM)
        );
    }
}
