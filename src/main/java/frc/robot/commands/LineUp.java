package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.MathUtil;

public class LineUp extends CommandBase{
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private PIDController xPid, txPid, distPid;
    private LimeLightSubsystem limeLightSubsystem;
    private AprilTagFieldLayout atfl;
    private double[] rPose;

  public LineUp(SwerveDriveSubsystem swerveDriveSubsystem, LimeLightSubsystem limeLightSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.limeLightSubsystem = limeLightSubsystem;
    addRequirements(swerveDriveSubsystem);
    SendableRegistry.addLW(this, "LineUp");
    rPose = new double[]{0,0,0};
  }

  @Override
  public void initialize() {
    xPid = new PIDController(0.00, 0, 0);
    txPid = new PIDController(0.01, 0.00001, 0);
    distPid = new PIDController(0.14, 0, 0);
    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      //  var alliance = DriverStation.getAlliance();
      //  atfl.setOrigin(alliance == Alliance.Blue ? //aprilTagFieldLayout
      //  OriginPosition.kBlueAllianceWallRightSide
      //  : OriginPosition.kRedAllianceWallRightSide);

    } catch (Exception e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      atfl = null;
    }
  }

  @Override
  public void execute() {
    //Shuffleboard.getTab("Preferences").add("transform", "transformedTarget.toString()");
    
    xPid.setSetpoint(0);
    txPid.setSetpoint(0);
    distPid.setSetpoint(-20);
    boolean aligned = false;
    if (!aligned) {
        rPose = limeLightSubsystem.getX_Z_Tx();
        
        if (limeLightSubsystem.getTy() <= -19 || limeLightSubsystem.getTy() == 0){
          if (limeLightSubsystem.getTx() != 0)
          {
            swerveDriveSubsystem.drive(new ChassisSpeeds(0, -xPid.calculate(limeLightSubsystem.getTx()),
                                        txPid.calculate(limeLightSubsystem.getTx()) * 2));
          }
          else{
            aligned = true;
            swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
            swerveDriveSubsystem.brake();
          }
        }
        else {
            swerveDriveSubsystem.drive(new ChassisSpeeds(-distPid.calculate(limeLightSubsystem.getTy()), -xPid.calculate(limeLightSubsystem.getTx()),
            txPid.calculate(limeLightSubsystem.getTx())));
        }
    }
  }
}

