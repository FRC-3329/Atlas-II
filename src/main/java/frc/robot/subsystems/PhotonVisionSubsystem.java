package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import frc.robot.constants.PVConstants;
import frc.robot.utils.VisionData.EstimateConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class PhotonVisionSubsystem extends SubsystemBase {
	private final String cameraName;
	private final PhotonCamera camera;
	private final PhotonPoseEstimator photonEstimator;
	private final EstimateConsumer estConsumer;
	private final String logName;

	private Matrix<N3, N1> curStdDevs;

	/**
	 * @param cameraName    name of the camera in PhotonVision
	 * @param robotToCamera physical transform from robot center to camera lens
	 * @param estConsumer   callback to feed pose estimates into the drivetrain
	 */
	public PhotonVisionSubsystem(String cameraName, Transform3d robotToCamera, EstimateConsumer estConsumer) {
		this.cameraName = cameraName;
		this.estConsumer = estConsumer;

		camera = new PhotonCamera(cameraName);
		logName = "PV/" + cameraName + "/";
		photonEstimator = new PhotonPoseEstimator(
				PVConstants.kTagLayout,
				robotToCamera);

		// Throttle FPS while disabled to save bandwidth; full speed when enabled
		camera.setFPSLimit(4);
		RobotModeTriggers.disabled()
				.onTrue(runOnce(() -> camera.setFPSLimit(4)).ignoringDisable(true))
				.onFalse(runOnce(() -> camera.setFPSLimit(0)));

		SmartDashboard.putData(cameraName + " Camera Fast", this.runOnce(() -> {
			camera.setFPSLimit(20);
		}).ignoringDisable(true));
	}

	/**
	 * Dynamically adjusts standard deviations based on tag count and distance
	 * to control how much the Kalman filter trusts each vision estimate.
	 */
	private void updateEstimationStdDevs(
			Optional<EstimatedRobotPose> estimatedPose,
			List<PhotonTrackedTarget> targets) {
		if (estimatedPose.isEmpty()) {
			curStdDevs = PVConstants.kSingleTagStdDevs;
		} else {
			Matrix<N3, N1> estStdDevs = PVConstants.kSingleTagStdDevs;
			int numTags = 0;
			double avgDist = 0;
			for (PhotonTrackedTarget tgt : targets) {
				Optional<Pose3d> tagPose = photonEstimator
						.getFieldTags()
						.getTagPose(tgt.getFiducialId());

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
										.get().estimatedPose.toPose2d()
										.getTranslation());
			}

			if (numTags == 0) {
				curStdDevs = PVConstants.kSingleTagStdDevs;
			} else {
				avgDist /= numTags;

				// Multiple tags give a more reliable solve
				if (numTags > 1) {
					estStdDevs = PVConstants.kMultiTagStdDevs;
				}

				// Single tag at long range is unreliable — reject it entirely
				if (numTags == 1 && avgDist > 4) {
					estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				} else {
					// Scale uncertainty with distance squared
					estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
				}

				curStdDevs = estStdDevs;
			}
		}
	}

	public Matrix<N3, N1> getEstimationStdDevs() {
		return curStdDevs;
	}

	public String getCameraName() {
		return cameraName;
	}

	@Override
	public void periodic() {
		if (camera.isConnected()) {
			DogLog.clearFault(cameraName);
		} else {
			DogLog.logFault(cameraName);
		}

		Optional<EstimatedRobotPose> visionEst = Optional.empty();
		for (PhotonPipelineResult change : camera.getAllUnreadResults()) {
			// Prefer multi-tag PnP for accuracy; single-tag fallback commented out
			visionEst = photonEstimator.estimateCoprocMultiTagPose(change);

			if (visionEst.isEmpty()) {
				// visionEst = photonEstimator.estimateLowestAmbiguityPose(change);
			}

			updateEstimationStdDevs(visionEst, change.getTargets());

			visionEst.ifPresent(est -> {
				Pose2d pose2d = est.estimatedPose.toPose2d();
				DogLog.log(logName + "pose", pose2d);
				Matrix<N3, N1> estStdDevs = getEstimationStdDevs();
				estConsumer.accept(
						est.estimatedPose, est.timestampSeconds, estStdDevs);
			});
		}
	}
}