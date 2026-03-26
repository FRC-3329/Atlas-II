package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    public LEDSubsystem() {
        led = new AddressableLED(LEDConstants.PWM_PORT);
        buffer = new AddressableLEDBuffer(LEDConstants.STRIP_LENGTH);

        led.setLength(LEDConstants.STRIP_LENGTH);
        led.start();

        setDefaultCommand(
                runPattern(LEDPattern.solid(Color.kBlack))
                        .withName("Off"));
    }

    @Override
    public void periodic() {
        led.setData(buffer);
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(buffer));
    }

    /**
     * Configures trigger-based LED patterns to communicate robot state to the driver.
     *
     * @param readyToShoot returns true when the turret is aimed and within range
     */
    public void configureLEDs(BooleanSupplier readyToShoot) {
        LEDPattern allianceSolid = (reader, writer) -> {
            Color c = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red
                    ? Color.kRed
                    : Color.kBlue;
            LEDPattern.solid(c).applyTo(reader, writer);
        };

        // Solid alliance color = ready to shoot
        new Trigger(readyToShoot)
                .whileTrue(runPattern(allianceSolid).withName("LEDReadyToShoot"));

        // Rainbow while disabled so the pit crew can tell the robot is powered on
        LEDPattern disabledPattern = LEDPattern.rainbow(255, 20)
                .scrollAtRelativeSpeed(edu.wpi.first.units.Units.Percent
                        .per(edu.wpi.first.units.Units.Second).of(25));
        RobotModeTriggers.disabled()
                .whileTrue(runPattern(disabledPattern)
                        .ignoringDisable(true)
                        .withName("LEDDisabled"));
    }
}
