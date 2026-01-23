package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QNConstants;
import frc.robot.utils.VisionData.EstimateConsumer;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/** Questnav subsystem for position tracking */
public class QuestNavSubsystem extends SubsystemBase {
    public final QuestNav questNav = new QuestNav();
    /**  Sends alerts over NetworkTables */
    private final Alert alert;

    /** Publisher for world pose to NetworkTables */
	private final StructPublisher<Pose3d> worldPosePublisher = NetworkTableInstance
        .getDefault() // default NT instance
        .getStructTopic("QuestNav/WorldPose", Pose3d.struct) // Topic name and struct
        .publish(new PubSubOption[0]); // Publish with default opts
    
    /**
     * Consumer that receives vision pose estimates from QN and feeds them to the robot's pose estimator.
     * The pose estimator (SwerveDrivePoseEstimator) fuses multiple data sources:
     *     - wheel odometry,
     *     - gyro readings
     *     - vision measurements 
     * from QN to produce a more accurate estimate of the robot's position on the field. 
     * Vision measurements help correct for drift that accumulates in wheel odometry over time. 
     * The STDDEV in QNConstants determine how much the estimator trusts QN data versus wheel odometry.
     */
    private final EstimateConsumer estimateConsumer;
    private boolean useEstimatedConsumer;

	private Optional<Pose3d> questWorldPose = Optional.empty();

    /** Create QuestNav Subsystem. */
    public QuestNavSubsystem(
        EstimateConsumer estimateConsumer,
        boolean useEstimatedConsumer
    ) {
        this.estimateConsumer = estimateConsumer;
        this.useEstimatedConsumer = useEstimatedConsumer;
        this.alert = new Alert("QN Not tracking!", Alert.AlertType.kWarning);
    }
    
    /** 
     * Sets whether the QN subsystem use the estimation consumer.
     * If true, the robot's position will be based on QN's pose estimation.
    */
    public void useEstimatedConsumer(boolean useEstimatedConsumer) {
        this.useEstimatedConsumer = useEstimatedConsumer;
    }

    /** @return Is QN currently tracking. */
    public boolean isTracking() {
        return questNav.isTracking();
    }

    /** @return is QN currently connected. */
    public boolean isConnected() {
        return questNav.isConnected();
    }

	/**
	 * Sets the Quest's position in worldspace coordinates. 
     * Applies the necessary robot to quest transformation.
	 * 
	 * @param pose The position in worldspace.
	 */
    public void setQuestPose(Pose3d pose) {
        Pose3d questPose = pose.transformBy(QNConstants.Robot_to_Quest);
        setQuestPoseRaw(questPose);
    }

	/**
	 * Sets the quest pose without transforming by the robot to quest offset.
	 * 
	 * @param pose pose to set the quest to.
	 */
    public void setQuestPoseRaw(Pose3d pose) {
        if (isConnected()) {
            questNav.setPose(pose);
        } else {
            DriverStation.reportError(
                "Failed to set QN position, appears to be not connected.", 
            true);
        }
    }

    /**
	 * Gets the position of the quest in worldspace. 
     * Applies the nessesary robot to quest transformation.
	 * 
	 * @return The Quest's worldspace position. Will be None if not tracking.
	 */
    public Optional<Pose3d> getQuestPose() {
        return getQuestPoseRaw()
            .map(pose -> pose.transformBy(
                QNConstants.Robot_to_Quest.inverse()
            ));

    }
	/**
     * Gets the quest pose without any transformation.
     * 
	 * @return The Quest's pose without any transformation. Will be None if not tracking.
	 */
    public Optional<Pose3d> getQuestPoseRaw() {
        periodic(); // Update

        if (questWorldPose.isEmpty()) {
            DriverStation.reportWarning(
                "Quest pose unavailable, potentially not tracking/connected", 
            true);
        }

        return questWorldPose;
    }

    @Override
    public void periodic() {
        questNav.commandPeriodic();
        alert.set(!isTracking());

        if (isTracking()) {
            // Get latest pose frames from Quest
            PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

            // Loop over pose frames and send to pose estimator
            for (PoseFrame frame : questFrames) {
                // Pose of quest
                Pose3d questPose = frame.questPose3d();
        
                // Update latest world pose
                questWorldPose = Optional.of(questPose);

                // Timestamp of when data was sent
                double timestamp = frame.dataTimestamp();
                // Transform by mount pose to get robot pose
                Pose3d robotPose = questPose.transformBy(QNConstants.Robot_to_Quest.inverse());

                // Add quest pos to be tracked seperately from the robot's estimator
                worldPosePublisher.accept(robotPose);

                // Add the measurement to the estimator
                if (useEstimatedConsumer) {
                    estimateConsumer.accept(robotPose, timestamp, QNConstants.STD_Devs);
                }
            }
        } else {
            // Clear pose
            questWorldPose = Optional.empty();
        }
    }
}
