package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants.VisionConstants;

public class Vision {
    private PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraName);

    public PhotonPipelineResult ProcessFrame(){
        // Read in relevant data from the Camera
        var result = camera.getLatestResult();
        return result;
    }

    public double targetYaw(int targetNumber){
        var result = ProcessFrame();
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == targetNumber) {
                    // Found Tag, record its information
                    return -target.getYaw();
                }
            }
        }
        return 0;
    }
}
