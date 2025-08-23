package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

public final class Vision extends SubsystemBase implements NiceSubsystem {
    private final VisionCamera camera;

    private final DoubleArrayPublisher estimatedPoseLogger;

    private final PhotonPoseEstimator poseEstimator;

    private final CommandSwerveDrivetrain drivetrain;

    private Matrix<N3, N1> currentStdDevs = Constants.VisionConstants.SINGLE_TAG_DEVIATION;

    public Vision(String CameraName, Transform3d position, CommandSwerveDrivetrain drivetrain) {
        // ON ACTUAL ROBOT YOU NEED TO PROVIDE ACTUAL VALUES TO REPRESENT THE CAMERA'S
        // POSITION
        // ex: Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0,
        // 0.5), new Rotation3d(0,0,0));
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.

        // translation3d takes in x , y , and z , rotation3d takes in roll, pitch, yaw

        camera = new VisionCamera(CameraName, position);

        this.drivetrain = drivetrain;

        final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(
                AprilTagFields.k2025ReefscapeAndyMark);

        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                position);

        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

        // logging crap
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        estimatedPoseLogger = inst.getDoubleArrayTopic(CameraName + "_estimated_pose").publish();
    }

    private Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        List<PhotonPipelineResult> results = camera.getCamera().getAllUnreadResults();
        Optional<EstimatedRobotPose> visionEstimate = Optional.empty();

        for (PhotonPipelineResult result : results) {
            visionEstimate = poseEstimator.update(result);

            updateEstimationStdDevs(visionEstimate, result.getTargets());
        }

        return visionEstimate;
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose,
            List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // if there is no pose present, default to single tag std devs
            currentStdDevs = Constants.VisionConstants.SINGLE_TAG_DEVIATION;

        } else {
            // Start running Heuristic
            Matrix<N3, N1> estimatedStandardDeviations = Constants.VisionConstants.SINGLE_TAG_DEVIATION;
            int numberOfTags = 0;
            double averageDistance = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (PhotonTrackedTarget target : targets) {
                Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());

                if (tagPose.isEmpty()) {
                    continue;
                }

                numberOfTags++;

                averageDistance += tagPose.get().toPose2d().getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            // No tags are visible
            if (numberOfTags == 0) {
                currentStdDevs = Constants.VisionConstants.SINGLE_TAG_DEVIATION;
            } else {
                // One or more tags visible, run the full heuristic.
                // take average
                averageDistance /= numberOfTags;

                // decrease standard deviations if multiple targets are present
                if (numberOfTags > 1)
                    estimatedStandardDeviations = Constants.VisionConstants.MULTI_TAG_DEVIATION;

                // increase standard deviations based upon average distance
                if (numberOfTags == 1 && averageDistance > 4) {
                    currentStdDevs = estimatedStandardDeviations;
                }

            }

        }

    }

    @Override
    public void periodic() {
        // RUN BEAST MODE POSE ESTIMATING SWAG :100:

        Optional<EstimatedRobotPose> estimatedRobotPose = getEstimatedGlobalPose();

        estimatedRobotPose.ifPresent(pose -> {
            drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                    Utils.fpgaToCurrentTime(pose.timestampSeconds), currentStdDevs);

            estimatedPoseLogger.set(new double[] {
                    pose.estimatedPose.getX(),
                    pose.estimatedPose.getY(),
                    pose.estimatedPose.getRotation().toRotation2d().getDegrees()
            });
        });
    }

    @Override
    public void initialize() {

    }

    public static class VisionCamera {
        private final PhotonCamera camera;
        private final Transform3d position;

        public VisionCamera(String cameraName, Transform3d position) {
            camera = new PhotonCamera(cameraName);
            this.position = position;
        }

        public PhotonCamera getCamera() {
            return camera;
        }

        public Transform3d getPosition() {
            return position;
        }
    }
}