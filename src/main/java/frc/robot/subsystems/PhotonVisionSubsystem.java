package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
    private PhotonCamera camera;
   
    public PhotonVisionSubsystem(){
        this.camera = new PhotonCamera("photonvision");



    }
    public PhotonCamera getCamera(){
        return camera;
    }
    

}
