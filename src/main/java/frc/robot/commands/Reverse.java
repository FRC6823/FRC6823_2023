package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pigeon2Handler;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class Reverse extends CommandBase{
    private SwerveDriveSubsystem swerve;
    private Pigeon2Handler pigeon;
    private Timer time;


    public Reverse(SwerveDriveSubsystem swerve, Pigeon2Handler pigeon)
    {
        addRequirements(swerve);
        this.swerve = swerve;
        this.pigeon = pigeon;
        time = new Timer();
        SendableRegistry.addLW(this, "Reverse");
    }

    @Override
    public void initialize(){
        time.reset();
        time.start();
    }

    @Override
    public void execute(){
        swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(3, 0, 0, pigeon.getAngleDeg()));
        
       SmartDashboard.putNumber("time", time.get());
    }

    public boolean isFinished(){
        if (time.hasElapsed(2)){
            swerve.drive(new ChassisSpeeds(0,0,0));
            swerve.brake();
            return true;
        }
        
        return false;
    }

}
