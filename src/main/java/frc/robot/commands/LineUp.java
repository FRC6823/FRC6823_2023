package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class LineUp extends CommandBase{
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private PIDController xPid, txPid, tyPid;
    private LimeLightSubsystem limeLightSubsystem;
    private String node;

  public LineUp(SwerveDriveSubsystem swerveDriveSubsystem, LimeLightSubsystem limeLightSubsystem, String node) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.limeLightSubsystem = limeLightSubsystem;
    addRequirements(swerveDriveSubsystem);
    SendableRegistry.addLW(this, "LineUp");
  }

  @Override
  public void initialize() {
    xPid = new PIDController(0.2, 0, 0);
    txPid = new PIDController(0.02, 0.000, 0);
    tyPid = new PIDController(0.14, 0, 0);

    if (node.equals("left")){
      
    }
    if (node.equals("right")){

    }
    if (node.equals("center")){

    }
  }

  @Override
  public void execute() {
    
    tyPid.setSetpoint(3);
    xPid.setSetpoint(.7);
    txPid.setSetpoint(0);

    boolean aligned = false;
    if (!aligned) {
        if (limeLightSubsystem.getTy() <= 3 || limeLightSubsystem.getTy() == 0){
          if (limeLightSubsystem.getTx() != 0 || limeLightSubsystem.get3dTX() != 0)
          {
            swerveDriveSubsystem.drive(new ChassisSpeeds(0, -xPid.calculate(limeLightSubsystem.get3dTX()) * 3,
                                        -txPid.calculate(limeLightSubsystem.get3dRY())));
          }
          else{
            aligned = true;
            swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
            swerveDriveSubsystem.brake();
          }
        }
        else {
            swerveDriveSubsystem.drive(new ChassisSpeeds(-tyPid.calculate(limeLightSubsystem.getTy()), -xPid.calculate(limeLightSubsystem.get3dTX()),
            -txPid.calculate(limeLightSubsystem.get3dRY())));
        }
    }
  }
}

