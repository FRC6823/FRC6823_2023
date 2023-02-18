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
    private AprilTagFieldLayout atfl;

  public LineUp(SwerveDriveSubsystem swerveDriveSubsystem, LimeLightSubsystem limeLightSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.limeLightSubsystem = limeLightSubsystem;
    addRequirements(swerveDriveSubsystem);
    SendableRegistry.addLW(this, "Follow Leader");
  }

  @Override
  public void initialize() {
    xPid = new PIDController(0.1, 0, 0);
    txPid = new PIDController(0.1, 0, 0);
    tyPid = new PIDController(0.2, 0, 0);
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
    
    xPid.setSetpoint(0.7);
    txPid.setSetpoint(0);
    tyPid.setSetpoint(-Math.PI/4.0);
    if (limeLightSubsystem.hasValidTarget()) {
        if (tyPid.calculate(limeLightSubsystem.getTy()) >= -0.1){
            swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
            swerveDriveSubsystem.brake();
        }
        else {
            swerveDriveSubsystem.drive(new ChassisSpeeds(-tyPid.calculate(limeLightSubsystem.getTy()), -xPid.calculate(limeLightSubsystem.getXYZ()[0]),
            txPid.calculate(limeLightSubsystem.getTx())));
        }
    }
  }
}

