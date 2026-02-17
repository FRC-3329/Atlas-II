package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FlywheelConstants.Hood;
import frc.robot.subsystems.FlywheelSubsystem;

/**
 * Command to zero the hood by slowly moving it down until a current spike is
 * detected.
 * This ensures the hood is at a known position even if it wasn't manually
 * zeroed.
 */
public class ZeroHoodCommand extends Command {
    private final FlywheelSubsystem flywheelSubsystem;
    private final Timer currentSpikeTimer;
    private boolean currentSpikeDetected;

    /**
     * @param flywheelSubsystem The flywheel subsystem that controls the hood
     */
    public ZeroHoodCommand(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.currentSpikeTimer = new Timer();

        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        currentSpikeDetected = false;
        currentSpikeTimer.restart();

        DogLog.log("ZeroHoodCommand/Status", "Starting hood zeroing");
    }

    @Override
    public void execute() {
        double current = flywheelSubsystem.getHoodCurrent();

        flywheelSubsystem.setHoodVoltage(Hood.ZEROING_VOLTAGE);

        if (current >= Hood.ZEROING_CURRENT_THRESHOLD) {
            if (!currentSpikeDetected) {
                currentSpikeDetected = true;
                currentSpikeTimer.reset();
                DogLog.log("ZeroHoodCommand/Status", "Current spike detected");
            }
        } else {
            currentSpikeDetected = false;
            currentSpikeTimer.reset();
        }

        DogLog.log("ZeroHoodCommand/Current", current);
        DogLog.log("ZeroHoodCommand/SpikeTimer", currentSpikeTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.stopHood();

        if (!interrupted) {
            flywheelSubsystem.zeroHoodPosition();
            DogLog.log("ZeroHoodCommand/Status", "Hood zeroed successfully");
        } else {
            DogLog.log("ZeroHoodCommand/Status", "Hood zeroing interrupted");
        }
    }

    @Override
    public boolean isFinished() {
        return currentSpikeDetected &&
                currentSpikeTimer.hasElapsed(Hood.ZEROING_CURRENT_SPIKE_DURATION);
    }
}
