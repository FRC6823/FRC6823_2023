package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.VecBuilder;
import frc.robot.Pigeon2Handler;
import frc.robot.subsystems.LimeLightSubsystem;



//std devs taken from Redux, professional estimators
public class PoseEstimatorSubsystem{
    SwerveDrivePoseEstimator estimator;
    Pigeon2Handler pigeon;
    LimeLightSubsystem limelight;
    SwerveDriveSubsystem swerveDrive;
    

    public PoseEstimatorSubsystem(SwerveDriveSubsystem swerveDrive, LimeLightSubsystem limelight, Pigeon2Handler pigeon, Pose2d initPose){
        this.swerveDrive = swerveDrive;
        this.pigeon = pigeon;
        this.limelight = limelight;
        estimator = new SwerveDrivePoseEstimator(swerveDrive.getKinematics(), pigeon.getAngleDeg(), swerveDrive.getSwerveModulePosition(), initPose, VecBuilder.fill(.1,.1,.1), VecBuilder.fill(.9,.9,.9));
    }

    public void addVisionMeasurement(Pose2d pose, double latency){
        
    }

    public Pose2d getPose(){
        return estimator.getEstimatedPosition();
    }

    public void periodic(){
        estimator.update(pigeon.getAngleRad(), swerveDrive.getSwerveModulePosition());
    }
}
