package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    private SendableChooser<Boolean> leftBias;
    private double[] setPts;
    private boolean strafe;

  public LineUp(SwerveDriveSubsystem swerveDrive, LimeLightSubsystem limeLight, Pigeon2Handler pigeon, String node) {
    this.swerveDrive = swerveDrive;
    this.limeLight = limeLight;
    this.pigeon = pigeon;
    addRequirements(swerveDrive);
    SendableRegistry.addLW(this, "LineUp");
    this.node = node;

    xPid = new PIDController(5, 0.001, 0.0);
    yawPid = new PIDController(Constants.yawKp, Constants.yawKi, 0);
    tzPid = new PIDController(4, 0, 0);

    yawPid.enableContinuousInput(0, 360);

    
    if (node.equals("left")){
      setPts = Constants.leftScore;
    }

    else if (node.equals("right")){
      setPts = Constants.rightScore;
    }

    else if (node.equals("pickup")){
      leftBias = new SendableChooser<Boolean>();
      leftBias.setDefaultOption("left", true);
      leftBias.addOption("right", false);
      Shuffleboard.getTab("Preferences").add("Pickup Bias", leftBias);

      setPts = new double[]{0,0,0};
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
    limeLight.lSetPipeline(0);
    limeLight.rSetPipeline(0);
    strafe = false;
  }

  @Override
  public void execute() {

    if (node.equals("pickup")){

      if (limeLight.lHasValidTarget() && (limeLight.lGetId() == 4 || limeLight.lGetId() == 5)){
        setPts = Constants.leftPickup;
        tzPid.setSetpoint(setPts[0]);
        xPid.setSetpoint(setPts[1]);
        yawPid.setSetpoint(setPts[2]);

        swerveDrive.drive(new ChassisSpeeds(MathUtil.clipToRange(tzPid.calculate(limeLight.lGet3dTZ()), 1), -MathUtil.clipToRange(xPid.calculate(limeLight.lGet3dTX()), 0.5), MathUtil.clipToRange(yawPid.calculate(pigeon.getYaw()), 1)));
      }

      else if (limeLight.rHasValidTarget() && (limeLight.rGetId() == 4 || limeLight.rGetId() == 5)){ 
        setPts = Constants.rightPickup;
        tzPid.setSetpoint(setPts[0]);
        xPid.setSetpoint(setPts[1]);
        yawPid.setSetpoint(setPts[2]);

        swerveDrive.drive(new ChassisSpeeds(MathUtil.clipToRange(tzPid.calculate(limeLight.rGet3dTZ()), 1), -MathUtil.clipToRange(xPid.calculate(limeLight.rGet3dTX()), 0.5), MathUtil.clipToRange(yawPid.calculate(pigeon.getYaw()), 1)));
      }

      else{
        //Seeking behavior if either limeLight doesn't detect pickup station tag
        if (leftBias.getSelected()){
          swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(1,1,yawPid.calculate(pigeon.getYaw()), pigeon.getAngleDeg()));
        }

        else{
          swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(1,-1,yawPid.calculate(pigeon.getYaw()), pigeon.getAngleDeg()));
        }
      }
    }




    else{
      if (node.equals("left")){

        if (limeLight.lHasValidTarget()){

          if (Math.abs(limeLight.lGet3dTX()) >= Constants.TX_MAX || Math.abs(limeLight.rGet3dTX()) <= Constants.TX_MAX){
            strafe = true;
          }

          else{
            strafe = false;
            swerveDrive.drive(new ChassisSpeeds(MathUtil.clipToRange(tzPid.calculate(limeLight.lGet3dTZ()), 0.75), -MathUtil.clipToRange(xPid.calculate(limeLight.lGet3dTX()), 1.25), MathUtil.clipToRange(yawPid.calculate(pigeon.getYaw()), 1)));
          }
        }

        else{

          if (!strafe)
            swerveDrive.brake();
        }

        if (strafe){
          swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 1, yawPid.calculate(pigeon.getYaw()), pigeon.getAngleDeg()));
        }
      }



      if (node.equals("right")){

        if (limeLight.rHasValidTarget()){

          if (Math.abs(limeLight.rGet3dTX()) >= Constants.TX_MAX || Math.abs(limeLight.lGet3dTX()) <= Constants.TX_MAX){
            strafe = true;
          }

          else{
            strafe = false;
            swerveDrive.drive(new ChassisSpeeds(MathUtil.clipToRange(tzPid.calculate(limeLight.rGet3dTZ()), 0.75), -MathUtil.clipToRange(xPid.calculate(limeLight.rGet3dTX()), 1.25), MathUtil.clipToRange(yawPid.calculate(pigeon.getYaw()), 1)));
          }
        }

        else{

        if (!strafe)
          swerveDrive.brake();
        }

        if(strafe){
          swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, -1, yawPid.calculate(pigeon.getYaw()), pigeon.getAngleDeg()));
        }
      }
    }

  }
}

