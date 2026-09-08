package frc.robot.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.photonvision.simulation.VisionSystemSim;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FlywheelConstants;
import frc.robot.constants.PVConstants;
import frc.robot.constants.SimulationConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/** Coordinates robot-wide simulation that spans more than one subsystem. */
public class RobotSimulation {
    private final SwerveSubsystem drivebase;
    private final FlywheelSubsystem flywheel;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;
    private final TurretSubsystem turret;

    private final VisionSystemSim visionSimulation = new VisionSystemSim("Robot");
    private final SimulatedArena arena = SimulatedArena.getInstance();
    private final AbstractDriveTrainSimulation driveSimulation;
    private final IntakeSimulation fuelIntakeSimulation;
    private final Timer fuelMovementTimer = new Timer();

    private int launchedFuelCount;
    private int ejectedFuelCount;

    public RobotSimulation(
            SwerveSubsystem drivebase,
            FlywheelSubsystem flywheel,
            IndexerSubsystem indexer,
            IntakeSubsystem intake,
            TurretSubsystem turret,
            PhotonVisionSubsystem... cameras) {
        this.drivebase = drivebase;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.intake = intake;
        this.turret = turret;

        visionSimulation.addAprilTags(PVConstants.kTagLayout);
        for (PhotonVisionSubsystem camera : cameras) {
            camera.addToSimulation(visionSimulation);
        }

        driveSimulation = drivebase.getSwerveDrive().getMapleSimDrive().orElseThrow();
        fuelIntakeSimulation = IntakeSimulation.OverTheBumperIntake(
                RebuiltFuelOnField.REBUILT_FUEL_INFO.type(),
                driveSimulation,
                SimulationConstants.Fuel.INTAKE_WIDTH,
                SimulationConstants.Fuel.INTAKE_EXTENSION,
                IntakeSide.FRONT,
                SimulationConstants.Fuel.CAPACITY);
        fuelIntakeSimulation.setGamePiecesCount(SimulationConstants.Fuel.PRELOAD_COUNT);
        arena.resetFieldForAuto();

        fuelMovementTimer.start();
        SmartDashboard.putData("Simulation/LoadFuel",
                Commands.runOnce(() -> fuelIntakeSimulation.addGamePieceToIntake())
                        .ignoringDisable(true));
        SmartDashboard.putData("Simulation/ClearStoredFuel",
                Commands.runOnce(() -> fuelIntakeSimulation.setGamePiecesCount(0))
                        .ignoringDisable(true));
    }

    public void periodic() {
        Pose2d robotPose = drivebase.getSimulationPose();
        visionSimulation.update(robotPose);

        if (intake.isDown() && intake.isRollerRunningForward()) {
            fuelIntakeSimulation.startIntake();
        } else {
            fuelIntakeSimulation.stopIntake();
        }

        if (fuelMovementTimer.hasElapsed(SimulationConstants.Fuel.SHOT_INTERVAL_SECONDS)) {
            if (indexer.isFeedingForward()
                    && Math.abs(flywheel.getVelocityMeasure().in(RPM))
                            >= SimulationConstants.Fuel.MIN_SHOOTING_RPM) {
                launchFuel(robotPose);
            } else if (indexer.isFeedingBackward() && intake.isRollerRunningBackward()) {
                ejectFuel(robotPose);
            }
        }

        Pose3d[] fuelPoses = arena.getGamePiecesArrayByType(
                RebuiltFuelOnField.REBUILT_FUEL_INFO.type());
        DogLog.log("Simulation/Field/Fuel", fuelPoses);
        DogLog.log("Simulation/StoredFuel", fuelIntakeSimulation.getGamePiecesAmount());
        DogLog.log("Simulation/LaunchedFuel", launchedFuelCount);
        DogLog.log("Simulation/EjectedFuel", ejectedFuelCount);
        DogLog.log("Simulation/Mechanisms", getMechanismPoses(robotPose));
    }

    private void launchFuel(Pose2d robotPose) {
        if (!fuelIntakeSimulation.obtainGamePieceFromIntake()) {
            return;
        }

        Rotation2d shooterFacing = robotPose.getRotation()
                .plus(Rotation2d.k180deg)
                .plus(Rotation2d.fromDegrees(turret.getAngleMeasure().in(Degrees)));
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                drivebase.getRobotVelocity(), robotPose.getRotation());
        double launchSpeedMetersPerSecond = Math.abs(
                flywheel.getVelocityMeasure().in(RotationsPerSecond))
                * Math.PI
                * FlywheelConstants.Flywheel.WHEEL_DIAMETER.in(edu.wpi.first.units.Units.Meters)
                * SimulationConstants.Fuel.LAUNCH_EFFICIENCY;
        Translation2d shooterOffsetInFacingFrame = TurretConstants.ROBOT_TO_TURRET
                .getTranslation()
                .rotateBy(robotPose.getRotation().minus(shooterFacing));

        arena.addGamePieceProjectile(new RebuiltFuelOnFly(
                robotPose.getTranslation(),
                shooterOffsetInFacingFrame,
                fieldRelativeSpeeds,
                shooterFacing,
                SimulationConstants.Fuel.SHOOTER_HEIGHT,
                MetersPerSecond.of(launchSpeedMetersPerSecond),
                flywheel.getHoodAngleMeasure()));
        launchedFuelCount++;
        fuelMovementTimer.restart();
    }

    private void ejectFuel(Pose2d robotPose) {
        if (!fuelIntakeSimulation.obtainGamePieceFromIntake()) {
            return;
        }

        double ejectDistanceMeters = driveSimulation.config.bumperLengthX
                .div(2.0)
                .plus(SimulationConstants.Fuel.INTAKE_EXTENSION)
                .in(edu.wpi.first.units.Units.Meters);
        Translation2d ejectOffset = new Translation2d(
                ejectDistanceMeters,
                0.0).rotateBy(robotPose.getRotation());
        arena.addGamePiece(new RebuiltFuelOnField(robotPose.getTranslation().plus(ejectOffset)));
        ejectedFuelCount++;
        fuelMovementTimer.restart();
    }

    private Pose3d[] getMechanismPoses(Pose2d robotPose) {
        Rotation2d shooterFacing = robotPose.getRotation()
                .plus(Rotation2d.k180deg)
                .plus(Rotation2d.fromDegrees(turret.getAngleMeasure().in(Degrees)));
        Translation2d turretTranslation = robotPose
                .transformBy(TurretConstants.ROBOT_TO_TURRET)
                .getTranslation();
        Pose3d shooterPose = new Pose3d(
                new Translation3d(
                        turretTranslation.getX(),
                        turretTranslation.getY(),
                        SimulationConstants.Fuel.SHOOTER_HEIGHT.in(edu.wpi.first.units.Units.Meters)),
                new Rotation3d(
                        0.0,
                        -flywheel.getHoodAngleMeasure().in(edu.wpi.first.units.Units.Radians),
                        shooterFacing.getRadians()));

        return new Pose3d[] { new Pose3d(robotPose), shooterPose };
    }

    public int getStoredFuelCount() {
        return fuelIntakeSimulation.getGamePiecesAmount();
    }

    public int getLaunchedFuelCount() {
        return launchedFuelCount;
    }

    public int getEjectedFuelCount() {
        return ejectedFuelCount;
    }

    public void addFuel() {
        fuelIntakeSimulation.addGamePieceToIntake();
    }
}
