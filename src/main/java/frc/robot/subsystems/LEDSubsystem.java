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

    /**
     * Creates a command that continuously applies a pattern to the LED buffer.
     *
     * @param pattern the LED pattern to run
     * @return a command that applies the pattern each cycle
     */
    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(buffer));
    }

    /**
     * Configures LED patterns and triggers.
     *
     * <ul>
     * <li><b>Ready to shoot</b> (turret on-target, auto-tracking, valid shot
     * distance):
     * solid alliance color (red or blue).</li>
     * <li><b>Disabled</b>: scrolling rainbow.</li>
     * </ul>
     * 
     * @param readyToShoot a supplier that returns true when the robot is ready to
     *                     shoot
     */
    public void configureLEDs(BooleanSupplier readyToShoot) {
        LEDPattern allianceSolid = (reader, writer) -> {
            Color c = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red
                    ? Color.kRed
                    : Color.kBlue;
            LEDPattern.solid(c).applyTo(reader, writer);
        };

        // Ready to shoot -> solid alliance color
        new Trigger(readyToShoot)
                .whileTrue(runPattern(allianceSolid).withName("LEDReadyToShoot"));

        // Disabled -> scrolling rainbow (8%~)
        LEDPattern disabledPattern = LEDPattern.rainbow(255, 20)
                .scrollAtRelativeSpeed(edu.wpi.first.units.Units.Percent
                        .per(edu.wpi.first.units.Units.Second).of(25));
        RobotModeTriggers.disabled()
                .whileTrue(runPattern(disabledPattern)
                        .ignoringDisable(true)
                        .withName("LEDDisabled"));
    }
}
