package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.MathUtil;

public class LineUp extends CommandBase{
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private PIDController xPid, txPid, tyPid;
    private LimeLightSubsystem limeLightSubsystem;
    private String node;
    private double[] setPts;
    private double tolerance;
    private boolean aligned;
    private boolean reflective;
    private boolean finished;

  public LineUp(SwerveDriveSubsystem swerveDriveSubsystem, LimeLightSubsystem limeLightSubsystem, String node) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.limeLightSubsystem = limeLightSubsystem;
    addRequirements(swerveDriveSubsystem);
    SendableRegistry.addLW(this, "LineUp");
    this.node = node;

    tyPid = new PIDController(0.1, 0, 0);
    xPid = new PIDController(0.4, 0, 0);
    txPid = new PIDController(0.07, 0.00001, 0);

    if (node.equals("left")){
      setPts = Constants.leftScore;
      tolerance = .15;
    }
    else if (node.equals("right")){
      setPts = Constants.rightScore;
      tolerance = .15;
    }
    else if (node.equals("center")){
      setPts = Constants.centerScore;
      tolerance = .1;
    }
    else if (node.equals("pickup")){
      setPts = Constants.pickup;
      tolerance = 0.1;
    }
    else {
      setPts = new double[]{0,0,0};
    }
  }

  @Override
  public void initialize() {
    aligned = false;
    reflective = false;
    finished = false;
    limeLightSubsystem.setPipeline(0);
  }

  @Override
  public void execute() {
    
    tyPid.setSetpoint(setPts[0]);
    xPid.setSetpoint(setPts[1]);
    txPid.setSetpoint(setPts[2]);
    
    if (!aligned) {
        if (MathUtil.clipToZero(txPid.calculate(limeLightSubsystem.get3dRY()), tolerance) != 0 || MathUtil.clipToZero(xPid.calculate(limeLightSubsystem.get3dTX()), tolerance) != 0 || MathUtil.clipToZero(tyPid.calculate(limeLightSubsystem.getTy()), tolerance) != 0){
          
          swerveDriveSubsystem.drive(new ChassisSpeeds(-tyPid.calculate(limeLightSubsystem.getTy()), -xPid.calculate(limeLightSubsystem.get3dTX()),
            -txPid.calculate(limeLightSubsystem.get3dRY())));
        }
        else {
          aligned = true;
          swerveDriveSubsystem.drive(new ChassisSpeeds(0,0,0));
          swerveDriveSubsystem.brake();

          if (node.equals("left") || node.equals("right")){
            limeLightSubsystem.setPipeline(1);
            reflective = true;
            swerveDriveSubsystem.drive(new ChassisSpeeds(0,0,0));
            swerveDriveSubsystem.brake();
          }
          else{
            finished = true;
          }
        }
    }

    if (reflective){
      limeLightSubsystem.setPipeline(1);
      tyPid.setSetpoint(0);
      txPid.setSetpoint(0);

      if (MathUtil.clipToZero(limeLightSubsystem.getTy(), 0.2) != 0 || MathUtil.clipToZero(limeLightSubsystem.getTx(), 0.2) != 0){
          swerveDriveSubsystem.drive(new ChassisSpeeds(-tyPid.calculate(limeLightSubsystem.getTy()), 0, 
                                                          txPid.calculate(limeLightSubsystem.getTx())));
        
      }
      else if (limeLightSubsystem.getTv() != 0){
        swerveDriveSubsystem.brake();
        limeLightSubsystem.setPipeline(0);
        finished = true;
      }
      else{
        swerveDriveSubsystem.drive(new ChassisSpeeds(0,0,0));
        swerveDriveSubsystem.brake();
      }
    }
  }

  public boolean isFinished(){
    return finished;
  }
}

