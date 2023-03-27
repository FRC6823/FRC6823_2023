package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
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
        yawPid = new PIDController(0.1, 0, 0);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        yawPid.setSetpoint(180);
    }

    @Override
    public void execute(){
        swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speed, 0, yawPid.calculate(pigeon.getPositiveYaw().getDegrees()), pigeon.getAngleDeg()));
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
