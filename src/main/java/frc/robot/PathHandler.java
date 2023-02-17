package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.Constants;

public class PathHandler {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private PathConstraints constraints;

    public PathHandler(SwerveDriveSubsystem swerveDriveSubsystem)
    {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        
        constraints = new PathConstraints(Constants.kMaxVelocity, Constants.kMaxAccel);
    }

    public Command TeleopScore(boolean dir){
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.kMaxVelocity, Constants.kMaxAccel);
        if(dir){
            trajectoryConfig.setReversed(true);
        }

        double x = swerveDriveSubsystem.getRobotPose().getX();
        double y = swerveDriveSubsystem.getRobotPose().getY();
        Rotation2d heading = swerveDriveSubsystem.getRobotPose().getRotation();

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory
            (new Pose2d(x, y, heading), 

            List.of(), 

             new Pose2d(x + 0.47, y, heading), trajectoryConfig);
        
        PIDController xController = new PIDController(.1, 0.000, 0);
        PIDController yController = new PIDController(.1, 0.000, 0);
        ProfiledPIDController turnController = new ProfiledPIDController(0.05, 0, 0, Constants.kTurnControlConstraints);
        turnController.enableContinuousInput(Math.PI, Math.PI);
     
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory, 
            swerveDriveSubsystem::getRobotPose, 
            swerveDriveSubsystem.getKinematics(), 
            xController, yController, turnController, 
            swerveDriveSubsystem::setSwerveModuleStates, 
            swerveDriveSubsystem);

        return new SequentialCommandGroup(new InstantCommand(() -> swerveDriveSubsystem.resetPose()), swerveControllerCommand, new InstantCommand(() -> swerveDriveSubsystem.brake()));
    }

    public Command simpleAuto(){
        PathPlannerTrajectory path = PathPlanner.loadPath("Simple Auto", constraints);
        
        return PPSwerveControlCommand(path, true).beforeStarting(new InstantCommand(() -> swerveDriveSubsystem.resetPose()));
    }


    public Command PPSwerveControlCommand(PathPlannerTrajectory path, boolean stopAtEnd){

        PIDController xController = new PIDController(.1, 0, 0);
        PIDController yController = new PIDController(.1, 0, 0);
        PIDController turnController = new PIDController(0.1, 0, 0);
        turnController.enableContinuousInput(Math.PI, Math.PI);

        Command swerveControllerCommand = new PPSwerveControllerCommand(
            path, swerveDriveSubsystem::getRobotPose, swerveDriveSubsystem.getKinematics(),
            xController, yController, turnController, 
            swerveDriveSubsystem::setSwerveModuleStates, swerveDriveSubsystem);
        
        if (stopAtEnd){
            swerveControllerCommand = swerveControllerCommand.andThen(new InstantCommand(() -> swerveDriveSubsystem.brake()));
        }

        return swerveControllerCommand;
    }

}
