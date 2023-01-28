package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class FollowLeader extends CommandBase {
    private SwerveDriveSubsystem swerveDriveSubsystem;    
    private PIDController xPid,yPid,RPid;
    private PhotonCamera camera;

    public FollowLeader(SwerveDriveSubsystem swerveDriveSubsystem){
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        addRequirements(swerveDriveSubsystem);

    }

    @Override
    public void initialize(){
      xPid = new PIDController(0, 0, 0);
      yPid = new PIDController(0.1, 0, 0);
      RPid = new PIDController(0, 0, 0);
      this.camera = new PhotonCamera("photonvision");

    }

    @Override
    public void execute(){
        
        

        Shuffleboard.getTab("Preferences").add("transform","transformedTarget.toString()");
        
        xPid.setSetpoint(0);
        yPid.setSetpoint(5);
        RPid.setSetpoint(180);
        PhotonPipelineResult latestResult = camera.getLatestResult();
        if(latestResult.hasTargets()){
          List<PhotonTrackedTarget>  targets = latestResult.targets;
          for(PhotonTrackedTarget target: targets){
            if(target.getFiducialId() ==1){
                Transform3d transformedTarget = target.getBestCameraToTarget();
                Shuffleboard.getTab("Preferences").add("transform",transformedTarget.toString());
                swerveDriveSubsystem.drive(xPid.calculate(transformedTarget.getX()), yPid.calculate(transformedTarget.getY()), RPid.calculate(transformedTarget.getRotation().getAngle()));
            }
          }
        }

    }
}
