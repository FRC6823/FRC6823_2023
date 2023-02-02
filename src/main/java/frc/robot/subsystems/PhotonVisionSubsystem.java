package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
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
