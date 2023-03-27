package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pigeon2Handler;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class Reverse extends CommandBase{
    private SwerveDriveSubsystem swerve;
    private Pigeon2Handler pigeon;
    private Timer timer;
    private PIDController yawPid;
    private double time, speed;


    public Reverse(SwerveDriveSubsystem swerve, Pigeon2Handler pigeon, double time, double speed)
    {
        addRequirements(swerve);
        this.swerve = swerve;
        this.pigeon = pigeon;
        timer = new Timer();
        this.time = time;
        this.speed = speed;
        yawPid = new PIDController(0.2, 0, 0);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        yawPid.setSetpoint(180);
    }

    @Override
    public void execute(){
        swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speed, 0, yawPid.calculate(pigeon.getYaw()), pigeon.getAngleDeg()));
    }

    public boolean isFinished(){
        if (timer.hasElapsed(time)){
            swerve.drive(new ChassisSpeeds(0,0,0));
            swerve.brake();
            return true;
        }
        return false;
    }

}
