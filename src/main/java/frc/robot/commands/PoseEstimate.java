package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public final class PoseEstimate extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision camera;
    private final PhotonPoseEstimator poseEstimator;

    private Matrix<N3, N1> currentStdDevs;

    private final DoubleArrayPublisher estimatedPoses;

    public PoseEstimate(Vision vision, CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.camera = vision;

        final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(
                AprilTagFields.k2025ReefscapeAndyMark);

        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                this.camera.getPosition());

        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

        // logging crap
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        estimatedPoses = inst.getDoubleArrayTopic(this.camera.getName() + "_estimated_pose").publish();

        addRequirements(vision, drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Optional<EstimatedRobotPose> estimatedRobotPose = getEstimatedGlobalPose();

        estimatedRobotPose.ifPresent(pose -> {
            drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                    Utils.fpgaToCurrentTime(pose.timestampSeconds), currentStdDevs);

            estimatedPoses.set(new double[] {
                    pose.estimatedPose.getX(),
                    pose.estimatedPose.getY(),
                    pose.estimatedPose.getRotation().toRotation2d().getDegrees()
            });
        });
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return camera == null || !camera.getCamera().isConnected();
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
}
