package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Optional;

import frc.robot.Constants;
import frc.robot.Constants.PVConstants;
import frc.robot.utils.VisionData.EstimateConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
	private final PhotonCamera camera;
	private final PhotonPoseEstimator photonEstimator;
	private final EstimateConsumer estConsumer;
	private final MedianFilter xFilter;
	private final MedianFilter yFilter;
	private final MedianFilter thetaFilter;
	private final Alert cameraDisconnectedAlert;
	private final Alert cameraNotTrackingAlert;
	private final MutTime timeSinceLastUpdatedPose = Seconds.mutable(0.0);

	private final StructPublisher<Pose2d> publisherRaw = NetworkTableInstance
        .getDefault()
		.getStructTopic("photonvision/WorldPoseRaw", Pose2d.struct)
        .publish();

	private final StructPublisher<Pose2d> publisherFiltered = NetworkTableInstance
        .getDefault()
		.getStructTopic("photonvision/WorldPoseFiltered", Pose2d.struct)
        .publish();

	private Matrix<N3, N1> curStdDevs;
	private Pose2d lastUpdatedPose;
	private boolean useEstConsumer;

	/**
	 * @param estConsumer    consumer to add the vision data to
	 * @param useEstConsumer whether to use the estimation consumer. Set to
	 *                       {@code false} if using QuestNav. Set to {@code true} to
	 *                       continually update the robot's pose with PV
	 *                       calculations.
	 */
	public PhotonVisionSubsystem(EstimateConsumer estConsumer, boolean useEstConsumer) {
		this.estConsumer = estConsumer;
		this.useEstConsumer = useEstConsumer;

		camera = new PhotonCamera(PVConstants.CAMERA_NAME);

		cameraDisconnectedAlert = new Alert("photonvision", "PV Not Connected!!", AlertType.kError);
		cameraNotTrackingAlert = new Alert("photonvision", "PV Not Tracking!!", AlertType.kWarning);

        /*
		    We will filter the pos data because we do not care about the phase delay added 
            (the robot will be stationary when the pos is used).
		    Also filtering because we could accidentally set out pos to a bad data pos if we are just grabbing the raw pos data.         
        */  
		xFilter = new MedianFilter(10);
		yFilter = new MedianFilter(10);
		thetaFilter = new MedianFilter(10);

		photonEstimator = new PhotonPoseEstimator(
            PVConstants.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PVConstants.ROBOT_TO_CAMERA
        );

		photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

		lastUpdatedPose = new Pose2d(1, 1, Rotation2d.kZero);
	}

	public void setUseEsimationConsumer(boolean useEstConsumer) {
		this.useEstConsumer = useEstConsumer;
	}

	/**
	 * Calculates new std devs.
     * This algorithm is a heuristic that creates dynamic standard deviations based on number of: 
     *     - tags
     *     - estimation strategy
     *     - distance 
     * from the tags.
	 
	 * @param estimatedPose The estimated pose to guess standard deviations for.
	 * @param targets       All targets in this camera frame
	 */
	private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose,
		List<PhotonTrackedTarget> targets
    ) {
		if (estimatedPose.isEmpty()) {
			// No pose input, default to single-tag std devs
			curStdDevs = PVConstants.kSingleTagStdDevs;
		} else {
			// Pose present, start running Heuristic
			var estStdDevs = PVConstants.kSingleTagStdDevs;
			int numTags = 0;
			double avgDist = 0;

			// Precalculation, see how many tags we found, and calculate an
			// average-distance metric
			for (var tgt : targets) {
				var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());

				if (tagPose.isEmpty()) {
                    continue;
                }

				numTags++;

				avgDist += tagPose
					.get()
					.toPose2d()
					.getTranslation()
					.getDistance(
                        estimatedPose
                            .get()
                            .estimatedPose.toPose2d()
                            .getTranslation()
                    );
			}

			if (numTags == 0) {
				// No tags visible, default to single-tag std devs
				curStdDevs = PVConstants.kSingleTagStdDevs;
			} else {
				// One or more tags visible, run the full heuristic
				avgDist /= numTags;

				// Decrease std devs if multiple targets are visible
				if (numTags > 1) {
					estStdDevs = PVConstants.kMultiTagStdDevs;
                }

				// Increase std devs based on (average) distance
				if (numTags == 1 && avgDist > 4) {
					estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                } else {
					estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                }

				curStdDevs = estStdDevs;
			}
		}
	}

	/**
	 * Returns the calculated world position of the robot. 
     * This value is filtered. 
     * Requires a tag to be seen within the last second to be valid.
	 * 
	 * @return {@link Optional#of(Pose2d)} if a tag was seen within the last second,
	 *         <p>
	 *         {@link Optional#empty()} if no tags seen within the last second.
	 */
	public Optional<Pose2d> getPoseOptional() {
		if (timeSinceLastUpdatedPose.gt(Seconds.zero())) {
			return Optional.of(lastUpdatedPose);
		} else {
			DriverStation.reportWarning("Tried to get PV pose but it was invalid!!", false);
			return Optional.empty();
		}
	}

	/**
	 * Returns the latest standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()},
     * for use with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. 
     * This should only be used when there are targets visible.
	 */
	public Matrix<N3, N1> getEstimationStdDevs() {
		return curStdDevs;
	}

	@Override
	public void periodic() {
		cameraDisconnectedAlert.set(!camera.isConnected());
		if (timeSinceLastUpdatedPose.gt(Seconds.zero())) {
			// camera has updated pose within the last second
			cameraNotTrackingAlert.set(false);
			timeSinceLastUpdatedPose.mut_minus(Constants.LOOP_TIME);
		} else {
			cameraNotTrackingAlert.set(true);
		}

		Optional<EstimatedRobotPose> visionEst = Optional.empty();
		for (var change : camera.getAllUnreadResults()) {
			visionEst = photonEstimator.update(change);

			if (useEstConsumer) {
				updateEstimationStdDevs(visionEst, change.getTargets());
			}

			visionEst.ifPresent(est -> {
				Pose2d pose2d = est.estimatedPose.toPose2d();

				publisherRaw.accept(pose2d);

			    if (useEstConsumer) {
					var estStdDevs = getEstimationStdDevs();

					estConsumer.accept(
                        est.estimatedPose, est.timestampSeconds, estStdDevs
                    );
				}

				lastUpdatedPose = new Pose2d(
					xFilter.calculate(pose2d.getX()),
					yFilter.calculate(pose2d.getY()),
					Rotation2d.fromRadians(
                        thetaFilter.calculate(pose2d.getRotation().getRadians())
                    )
                );
                
				timeSinceLastUpdatedPose.mut_replace(PVConstants.VALID_TIME);
				publisherFiltered.accept(lastUpdatedPose);
		    });
		}
	}
}