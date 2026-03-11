package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
}
