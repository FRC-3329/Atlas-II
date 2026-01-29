package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FlywheelConstants;

public class FlywheelSubsystem extends SubsystemBase {
    private final TalonFX left, right;
    private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
    private final InterpolatingDoubleTreeMap map;
    private final Supplier<Pose2d> robotPoseSupplier;

    public static final String RPM_KEY = "Flywheel/RPM";
    public static final String AT_SPEED_KEY = "Flywheel/AtSpeed";
    public static final String SPEED_KEY = "Flywheel/Speed";

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

        // Convert distance from hub to RPM
        map = new InterpolatingDoubleTreeMap();
        // TODO: Fill proper values
        map.put(1.0, 1000.0);
        map.put(2.0, 2000.0);

        // Set default command to idle
        setDefaultCommand(
            this.runOnce(() -> {
                left.set(0);
                right.set(0);
            }).andThen(
                this.idle()
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

    /** Check if the flywheel is at the target speed */
    public boolean isAtSpeed() {
        boolean leftAtSpeed = left.getMotionMagicAtTarget().getValue();
        boolean rightAtSpeed = right.getMotionMagicAtTarget().getValue();

        return leftAtSpeed && rightAtSpeed;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean(AT_SPEED_KEY, isAtSpeed());
        SmartDashboard.putNumber(
            RPM_KEY,
            left
                .getVelocity()
                .getValue()
                .in(edu.wpi.first.units.Units.RPM)
        );
        SmartDashboard.putNumber(SPEED_KEY, left.get());
    }
}
