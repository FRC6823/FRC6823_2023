package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;

public class FollowLeader extends CommandBase {
  private SwerveDriveSubsystem swerveDriveSubsystem;
  private PIDController xPid, yPid, RPid;
  private PhotonCamera camera;
  private PhotonPoseEstimator photonPoseEstimator;
  private AprilTagFieldLayout atfl;

  public FollowLeader(SwerveDriveSubsystem swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    addRequirements(swerveDriveSubsystem);

  }

  @Override
  public void initialize() {
    xPid = new PIDController(0, 0, 0);
    yPid = new PIDController(0.1, 0, 0);
    RPid = new PIDController(0, 0, 0);
    this.camera = new PhotonCamera("photonvision");
    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      // var alliance = DriverStation.getAlliance();
      // aprilTagFieldLayout.setOrigin(alliance == Alliance.Blue ?
      // OriginPosition.kBlueAllianceWallRightSide
      // : OriginPosition.kRedAllianceWallRightSide);

    } catch (Exception e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      atfl = null;
    }

    Transform3d robotToCamera = new Transform3d(new Translation3d(.3, 0, 0.2),
        new Rotation3d(0, 0, 0));
    photonPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, this.camera,
        robotToCamera);

  }

  @Override
  public void execute() {

    Shuffleboard.getTab("Preferences").add("transform", "transformedTarget.toString()");

    xPid.setSetpoint(0);
    yPid.setSetpoint(5);
    RPid.setSetpoint(180);
    PhotonPipelineResult latestResult = camera.getLatestResult();
    if (latestResult.hasTargets()) {
      List<PhotonTrackedTarget> targets = latestResult.targets;
      for (PhotonTrackedTarget target : targets) {
        if (target.getFiducialId() == 1) {
          Transform3d transformedTarget = target.getBestCameraToTarget();
          Shuffleboard.getTab("Preferences").add("transform", transformedTarget.toString());
          swerveDriveSubsystem.drive(xPid.calculate(transformedTarget.getX()), yPid.calculate(transformedTarget.getY()),
              RPid.calculate(transformedTarget.getRotation().getAngle()));
        }
      }
    }

  }
}
