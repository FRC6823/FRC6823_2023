package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pigeon2Handler;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.Constants;


public class LineUp extends CommandBase{
    private SwerveDriveSubsystem swerveDrive;
    private PIDController tyPid, txPid, xPid, yawPid, tzPid;
    private LimeLightSubsystem limeLight;
    private Pigeon2Handler pigeon;
    private String node;
    private double[] setPts;
    private double tolerance;
    private boolean reflective;
    private boolean finished;

  public LineUp(SwerveDriveSubsystem swerveDrive, LimeLightSubsystem limeLight, Pigeon2Handler pigeon, String node) {
    this.swerveDrive = swerveDrive;
    this.limeLight = limeLight;
    this.pigeon = pigeon;
    addRequirements(swerveDrive);
    SendableRegistry.addLW(this, "LineUp");
    this.node = node;

    tyPid = new PIDController(0.1, 0, 0);
    txPid = new PIDController(0.1, 0, 0);
    xPid = new PIDController(4, 0, 0);
    yawPid = new PIDController(0.3, 0, 0);
    tzPid = new PIDController(2.5, 0, 0);

    if (node.equals("left")){
      setPts = Constants.leftScore;
      tolerance = .05;
    }
    else if (node.equals("right")){
      setPts = Constants.rightScore;
      tolerance = .05;
    }
    else if (node.equals("center")){
      setPts = Constants.centerScore;
      tolerance = .01;
    }
    else if (node.equals("pickup")){
      setPts = Constants.pickup;
      tolerance = 0.01;
    }
    else {
      setPts = new double[]{0,0,0};
    }
  }

  @Override
  public void initialize() {
    reflective = false;
    finished = false;
    limeLight.setPipeline(0);
  }

  @Override
  public void execute() {
    
    if (!reflective){

      tzPid.setSetpoint(setPts[0]);
      xPid.setSetpoint(setPts[1]);
      yawPid.setSetpoint(setPts[2]);

      tzPid.setTolerance(tolerance);
      xPid.setTolerance(tolerance);
      yawPid.setTolerance(1);

      if (!tzPid.atSetpoint() || !xPid.atSetpoint() || !yawPid.atSetpoint()){
          swerveDrive.drive(new ChassisSpeeds(tzPid.calculate(limeLight.get3dTZ()), -xPid.calculate(limeLight.get3dTX()), yawPid.calculate(pigeon.getYaw())));
      } 
      else{
          if (node.equals("left") || node.equals("right")){
              reflective = true;
          }
          else {
              finished = true;
          }
      }
    }
    else {
        limeLight.setPipeline(1); 
        txPid.setSetpoint(0);
        tyPid.setSetpoint(0);
        txPid.setTolerance(0.1);
        tyPid.setTolerance(0.1);

        if (!txPid.atSetpoint() || !yawPid.atSetpoint()){
            swerveDrive.drive(new ChassisSpeeds(tyPid.calculate(limeLight.getTy()), txPid.calculate(limeLight.getTx()), yawPid.calculate(pigeon.getYaw())));
        }
        else{
            swerveDrive.drive(new ChassisSpeeds(0, 0, 0));
            limeLight.setPipeline(0);
            finished = true;
        }
    }
  }
}

