package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pigeon2Handler;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.MathUtil;


public class LineUp extends CommandBase{
    private SwerveDriveSubsystem swerveDrive;
    private PIDController xPid, yawPid, tzPid;
    private LimeLightSubsystem limeLight;
    private Pigeon2Handler pigeon;
    private String node;
    private double[] setPts;

  public LineUp(SwerveDriveSubsystem swerveDrive, LimeLightSubsystem limeLight, Pigeon2Handler pigeon, String node) {
    this.swerveDrive = swerveDrive;
    this.limeLight = limeLight;
    this.pigeon = pigeon;
    addRequirements(swerveDrive);
    SendableRegistry.addLW(this, "LineUp");
    this.node = node;

    xPid = new PIDController(10, 0.001, 0.0);
    yawPid = new PIDController(Constants.yawKp, Constants.yawKi, 0);
    tzPid = new PIDController(3, 0, 0);

    yawPid.enableContinuousInput(0, 360);

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

    tzPid.setSetpoint(setPts[0]);
    xPid.setSetpoint(setPts[1]);
    yawPid.setSetpoint(setPts[2]);
  }

  @Override
  public void initialize() {
    limeLight.setPipeline(0);
  }

  @Override
  public void execute() {
    if (yawPid.getSetpoint() == 0){
      swerveDrive.drive(new ChassisSpeeds(MathUtil.clipToRange(tzPid.calculate(limeLight.get3dTZ()), 1), -MathUtil.clipToRange(xPid.calculate(limeLight.get3dTX()), 0.5), MathUtil.clipToRange(yawPid.calculate(pigeon.getYaw180()), 1)));
    }
    else{
      swerveDrive.drive(new ChassisSpeeds(MathUtil.clipToRange(tzPid.calculate(limeLight.get3dTZ()), 1), -MathUtil.clipToRange(xPid.calculate(limeLight.get3dTX()), 1.25), MathUtil.clipToRange(yawPid.calculate(pigeon.getYaw()), 1)));
    }
    
  }
}

