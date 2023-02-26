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
    txPid = new PIDController(0.01, 0, 0);
    distPid = new PIDController(0.07, 0, 0);
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
    int running = 0;
    int driving = 0;
    while (!aligned) {
        rPose = limeLightSubsystem.getX_Z_Tx();
        running++;
        SmartDashboard.putNumber("is running?", running);
        
        if (distPid.calculate(limeLightSubsystem.getTy()) >= 0.1){
          //if (MathUtil.clipToZero(xPid.calculate(limeLightSubsystem.getTx()), 0.1) != 0 && MathUtil.clipToZero(txPid.calculate(limeLightSubsystem.getTx()), 0.1) != 0)
          //{
            swerveDriveSubsystem.drive(new ChassisSpeeds(0, -xPid.calculate(limeLightSubsystem.getTx()),
            txPid.calculate(limeLightSubsystem.getTx())));
          //}
          //else{
          aligned = true;
          swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
          swerveDriveSubsystem.brake();
          //}
        }
        else {
            driving++;
            SmartDashboard.putNumber("driving", driving);
            SmartDashboard.putNumber("Drive value", -distPid.calculate(limeLightSubsystem.getTy()));
            swerveDriveSubsystem.drive(new ChassisSpeeds(-distPid.calculate(limeLightSubsystem.getTy()), -xPid.calculate(limeLightSubsystem.getTx()),
            txPid.calculate(limeLightSubsystem.getTx())));
        }
    }
  }
}

