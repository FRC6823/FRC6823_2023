package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonCameraSubsystem extends SubsystemBase {

    private PhotonCamera photonCamera;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    private double lastKnownZ;
    private double lastKnownX;

    public PhotonCameraSubsystem(String cameraName) {
        photonCamera = new PhotonCamera(cameraName);
        SendableRegistry.addLW(this, "Photon Camera Subsystem");
    }

    // "T"x is for theta, aka this is .
    public double getTx() {
        if (hasTarget())
            return target.getYaw();
        else return 0;
    }

    public double getTxRad() {
        if (hasTarget())
            return (target.getYaw() / 360.0) * (2.0 * Math.PI);
        else return 0;
    }

    public double getTy() {
        if (hasTarget())
            return target.getPitch();
        else return 0;
    }

    public double getTyRad() {
        if (hasTarget())
            return (target.getPitch() / 360.0) * (2.0 * Math.PI);
        else return 0;
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }

    @Override
    public void periodic() {
        result = photonCamera.getLatestResult();
        target = result.getBestTarget();
        SmartDashboard.putNumber("Photon tx", getTx());
        SmartDashboard.putNumber("Photon ty", getTy());
    }
}