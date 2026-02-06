package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final PowerDistribution pdh;

    public LEDs() {
        pdh = new PowerDistribution();
        pdh.clearStickyFaults();
    }

    /**
     * Whether to turn on or off the LEDs
     * true = on | false = off
     * @param state
     */
    public void set(boolean state) {
        pdh.setSwitchableChannel(state);
    }

    @Override
    public void periodic() {}
}
