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
    private PIDController xPid, txPid, distPid;
    private LimeLightSubsystem limeLightSubsystem;
    private AprilTagFieldLayout atfl;
    private double[] rPose;

  public LineUp(SwerveDriveSubsystem swerveDriveSubsystem, LimeLightSubsystem limeLightSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.limeLightSubsystem = limeLightSubsystem;
    addRequirements(swerveDriveSubsystem);
    SendableRegistry.addLW(this, "Follow Leader");
    rPose = new double[3];
  }

  @Override
  public void initialize() {
    xPid = new PIDController(0.1, 0, 0);
    txPid = new PIDController(0.1, 0, 0);
    distPid = new PIDController(0.2, 0, 0);
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
    distPid.setSetpoint(1);
    txPid.setSetpoint(0);
    if (limeLightSubsystem.hasValidTarget()) {
        rPose = limeLightSubsystem.getX_Z_Tx();

        if (distPid.calculate(rPose[1]) >= 0.1){
          swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
          swerveDriveSubsystem.brake();
        }
        
        if (limeLightSubsystem.getTy() < -20){
          rPose[1] = 1;
        }

        else {
            swerveDriveSubsystem.drive(new ChassisSpeeds(distPid.calculate(rPose[1]), -xPid.calculate(rPose[0]),
            txPid.calculate(rPose[2])));
        }
    }
  }
}

