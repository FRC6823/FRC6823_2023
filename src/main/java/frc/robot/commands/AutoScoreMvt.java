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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.Constants;

public class AutoScoreMvt {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private boolean dir; //true - left, false - right

    public AutoScoreMvt(SwerveDriveSubsystem swerveDriveSubsystem)
    {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        dir = false;
    }

    public void setDir(boolean direction){
        dir = direction;
    }

    public Command execute(){
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.kMaxSpeed, Constants.kMaxAccel);
        //trajectoryConfig.setReversed(true);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory
            (new Pose2d(0, 0, new Rotation2d(0.0)), 
            List.of(new Translation2d(3,0)), 
             new Pose2d(1, 0, new Rotation2d(0.0)), trajectoryConfig);
        
        PIDController xController = new PIDController(.5, 0.00001, 0);
        PIDController yController = new PIDController(.5, 0.00001, 0);
        ProfiledPIDController turnController = new ProfiledPIDController(0.1, 0, 0, Constants.kTurnControlConstraints);
        turnController.enableContinuousInput(Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveDriveSubsystem.getRobotPose(), swerveDriveSubsystem.getKinematics(), xController, yController, turnController, swerveDriveSubsystem::setSwerveModuleStates, swerveDriveSubsystem);

        return new SequentialCommandGroup (
            new InstantCommand(() -> swerveDriveSubsystem.resetPose()),
            swerveControllerCommand,
            new InstantCommand(() -> {swerveDriveSubsystem.brake();})
        );
    }
}
