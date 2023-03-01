package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.Constants;

public class LineUp extends CommandBase{
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private PIDController xPid, txPid, tyPid;
    private LimeLightSubsystem limeLightSubsystem;
    private String node;
    private double[] setPts;
    private boolean aligned;
    private boolean reflective;
    private boolean finished;

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
      setPts = Constants.leftScore;
    }
    else if (node.equals("right")){
      setPts = Constants.rightScore;
    }
    else if (node.equals("center")){
      setPts = Constants.centerScore;
    }
    else if (node.equals("pickup")){
      setPts = Constants.pickup;
    }
    else {
      setPts = new double[]{0,0,0};
    }

    aligned = false;
    reflective = false;
    finished = false;
  }

  @Override
  public void execute() {
    
    tyPid.setSetpoint(setPts[0]);
    xPid.setSetpoint(setPts[1]);
    txPid.setSetpoint(setPts[2]);
    
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

            if (node.equals("left") || node.equals("right")){
              limeLightSubsystem.setPipeline(1);
              reflective = true;
            }
            else{
              finished = true;
            }
          }
        }
        else {
            swerveDriveSubsystem.drive(new ChassisSpeeds(-tyPid.calculate(limeLightSubsystem.getTy()), -xPid.calculate(limeLightSubsystem.get3dTX()),
            -txPid.calculate(limeLightSubsystem.get3dRY())));
        }
    }

    if (reflective){
      tyPid.setSetpoint(0);
      txPid.setSetpoint(0);

      if (limeLightSubsystem.getTy() != 0 || limeLightSubsystem.getTx() != 0){
        swerveDriveSubsystem.drive(new ChassisSpeeds(-tyPid.calculate(limeLightSubsystem.getTy()), 0, 
                                                    -txPid.calculate(limeLightSubsystem.getTx())));
      }
      else{
        swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        swerveDriveSubsystem.brake();
        finished = true;
      }
    }
  }

  public boolean isFinished(){
    return finished;
  }
}

