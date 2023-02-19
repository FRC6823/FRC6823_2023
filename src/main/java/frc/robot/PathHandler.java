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

    public Command simpleAuto(){
        PathPlannerTrajectory path = PathPlanner.loadPath("Simple Auto", constraints);
        
        return PPSwerveControlCommand(path, false).beforeStarting(new InstantCommand(() -> swerveDriveSubsystem.resetPose()));
    }

    public Command balanceAuto(){
        PathPlannerTrajectory path = PathPlanner.loadPath("Balance Auto", constraints);
        
        return PPSwerveControlCommand(path, false).beforeStarting(new InstantCommand(() -> swerveDriveSubsystem.setPose(1.88, 3.27, 0)));
    }


    public Command PPSwerveControlCommand(PathPlannerTrajectory path, boolean stopAtEnd){

        PIDController xController = new PIDController(Constants.kP, 0, 0);
        PIDController yController = new PIDController(Constants.kP, 0, 0);
        PIDController turnController = new PIDController(Constants.kPThetaController, Constants.kIThetaController, Constants.kDThetaController);
        turnController.enableContinuousInput(Math.PI, Math.PI);

        Command PPswerveControllerCommand = new PPSwerveControllerCommand(
            path, swerveDriveSubsystem::getRobotPose, swerveDriveSubsystem.getKinematics(),
            xController, yController, turnController, 
            swerveDriveSubsystem::setSwerveModuleStates, swerveDriveSubsystem);
        
        if (stopAtEnd){
            PPswerveControllerCommand = PPswerveControllerCommand.andThen(new InstantCommand(() -> swerveDriveSubsystem.brake()));
        }

        return PPswerveControllerCommand;
    }

}
