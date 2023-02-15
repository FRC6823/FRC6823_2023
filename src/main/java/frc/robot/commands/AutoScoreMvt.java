package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.Constants;

public class AutoScoreMvt extends CommandBase{
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private boolean dir; //true - right, false - left
    private SwerveControllerCommand swerveControllerCommand;

    public AutoScoreMvt(SwerveDriveSubsystem swerveDriveSubsystem, boolean direction)
    {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        dir = direction;
    }

    public void initialize(){
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.kMaxSpeed, Constants.kMaxAccel);
        if(dir){
            trajectoryConfig.setReversed(true);
        }

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory
            (new Pose2d(swerveDriveSubsystem.getRobotPose().getX(), swerveDriveSubsystem.getRobotPose().getY(), swerveDriveSubsystem.getRobotPose().getRotation()), 

            List.of(), 

             new Pose2d(swerveDriveSubsystem.getRobotPose().getX(), swerveDriveSubsystem.getRobotPose().getY() + .47, swerveDriveSubsystem.getRobotPose().getRotation()), trajectoryConfig);
        
        PIDController xController = new PIDController(.5, 0.00001, 0);
        PIDController yController = new PIDController(.5, 0.00001, 0);
        ProfiledPIDController turnController = new ProfiledPIDController(0.1, 0, 0, Constants.kTurnControlConstraints);
        turnController.enableContinuousInput(Math.PI, Math.PI);

        swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveDriveSubsystem::getRobotPose, swerveDriveSubsystem.getKinematics(), xController, yController, turnController, swerveDriveSubsystem::setSwerveModuleStates, swerveDriveSubsystem);
    }

    public Command getMvt()
    {
        return new SequentialCommandGroup(swerveControllerCommand, new InstantCommand(() -> swerveDriveSubsystem.brake()));
    }
}
