package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDHSubsystem extends SubsystemBase {
    private final PowerDistribution pdh;

    public PDHSubsystem() {
        pdh = new PowerDistribution();
        pdh.clearStickyFaults();

        DogLog.setPdh(pdh);
    }

    /**
     * Whether to turn on or off the PDH
     * true = on | false = off
     * @param state
     */
    public void set(boolean state) {
        pdh.setSwitchableChannel(state);
    }

    @Override
    public void periodic() {}
}
