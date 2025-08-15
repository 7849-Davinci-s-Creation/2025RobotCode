package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public final class Vision extends SubsystemBase implements NiceSubsystem {
    private final PhotonCamera camera;
    private final Transform3d position;

    public Vision(String cameraName, Transform3d position) {
        // ON ACTUAL ROBOT YOU NEED TO PROVIDE ACTUAL VALUES TO REPRESENT THE CAMERA'S
        // POSITION
        // ex: Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0,
        // 0.5), new Rotation3d(0,0,0));
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.

        // translation3d takes in x , y , and z , rotation3d takes in roll, pitch, yaw
        camera = new PhotonCamera(cameraName);
        this.position = position;
    }

    public PhotonCamera getCamera() {
        return this.camera;
    }

    public Transform3d getPosition() {
        return this.position;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void initialize() {

    }
}