package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;

import java.util.HashMap;

public final class Vision extends SubsystemBase implements NiceSubsystem {
    private static Vision instance;

    private final VisionCam[] cameras;

    private final AprilTagFieldLayout aprilTagFieldLayout;

    private final HashMap<String, VisionCam> cameraMap;

    private Vision() {
        // ON ACTUAL ROBOT YOU NEED TO PROVIDE ACTUAL VALUES TO REPRESENT THE CAMERA'S
        // POSITION
        // ex: Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0,
        // 0.5), new Rotation3d(0,0,0));
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.

        // translation3d takes in x , y , and z , rotation3d takes in roll, pitch, yaw
        final PhotonCamera frontLeftPhotonCamera = new PhotonCamera(Constants.VisionConstants.FRONT_LEFT_CAMERA_NAME);
        final Transform3d frontLeftCameraPosition = new Transform3d(new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0));

        final PhotonCamera frontRightPhotonCamera = new PhotonCamera(Constants.VisionConstants.FRONT_RIGHT_CAMERA_NAME);
        final Transform3d frontRightCameraPosition = new Transform3d(new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0));

        final PhotonCamera backLeftPhotonCamera = new PhotonCamera(Constants.VisionConstants.BACK_LEFT_CAMERA_NAME);
        final Transform3d backLeftCameraPosition = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

        final PhotonCamera backRightPhotonCamera = new PhotonCamera(Constants.VisionConstants.BACK_RIGHT_CAMERA_NAME);
        final Transform3d backRightCameraPosition = new Transform3d(new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0));

        cameras = new VisionCam[4];

        cameras[0] = new VisionCam(frontLeftPhotonCamera, frontLeftCameraPosition);
        cameras[1] = new VisionCam(frontRightPhotonCamera, frontRightCameraPosition);
        cameras[2] = new VisionCam(backLeftPhotonCamera, backLeftCameraPosition);
        cameras[3] = new VisionCam(backRightPhotonCamera, backRightCameraPosition);

        cameraMap = new HashMap<>();
        // check that cameras are instantiated correctly, and that they are connected.
        for (VisionCam camera : cameras) {
            if (camera == null) {
                DriverStation.reportError("Initializing a camera created a null entry!", true);
                continue;
            } else {
                cameraMap.put(camera.camera.getName(), camera);
            }

            if (!camera.camera.isConnected()) {
                DriverStation.reportError("Camera: " + camera.camera.getName() + " is not connected!", true);
            }
        }

        // HOME FIELD IS ANDYMARK, COMP FIELD IS WELDED
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }

        return instance;
    }

    public VisionCam getCamera(String cameraName) {
        return cameraMap.get(cameraName);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void initialize() {
        PortForwarder.add(5800, "photonvision.local", 5800);
    }

    public AprilTagFieldLayout getAprilTagFieldLayout() {
        return aprilTagFieldLayout;
    }

    public VisionCam[] getCameras() {
        return cameras;
    }

    public record VisionCam(PhotonCamera camera, Transform3d cameraPosition) {

    }
}