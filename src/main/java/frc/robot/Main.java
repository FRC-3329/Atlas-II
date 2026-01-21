package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

// You should never modify this file
public final class Main {
    private Main() {
    }

    // Start robot lowk
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
