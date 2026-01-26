package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;

public class CalibrateQuestCommand extends Command {
    private record PointData(
        Translation3d position,
        Rotation3d robotHeading
    ) {}

    
}
