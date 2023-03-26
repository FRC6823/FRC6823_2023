package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pigeon2Handler;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class FreshmanCodeCommand extends CommandBase{
    
    //robot goes backwards for 3 seconds relative to position on field -Yaw
    private SwerveDriveSubsystem swerveDrive;
    private Pigeon2Handler pig;
    private Timer tim;

    public FreshmanCodeCommand(SwerveDriveSubsystem swerve, Pigeon2Handler pige){
        swerveDrive = swerve;
        pig = pige;
        tim = new Timer();
        addRequirements(swerve);
    }

    public void initialize(){
        tim.reset();
        tim.start();
    }

    @Override
    public void execute(){
        swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(2, 0, 0, pig.getAngleDeg()));
    }

    public boolean isFinished(){
        return tim.hasElapsed(3);
    }

}
