package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pigeon2Handler;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.MathUtil;

public class Unbalance extends CommandBase{
    private boolean balanced;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private Pigeon2Handler pigeon2;
    private PIDController pid;

    public Unbalance(Pigeon2Handler pigeon2, SwerveDriveSubsystem swerveDriveSubsystem){
        this.pigeon2 = pigeon2;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        pid = new PIDController(.2, 0, 0);
    }

    public void initialize(){
        balanced = false;
        pid.setSetpoint(10.5);
    }

    @Override
    public void execute(){
        if (MathUtil.clipToZero(pigeon2.getPitch() - 11, 0.5) == 0){
            balanced = true;
        }
        else if (!balanced){
            swerveDriveSubsystem.drive(new ChassisSpeeds(-pid.calculate(pigeon2.getPitch()), 0, 0));
        }
    }

    public boolean isFinished(){
        return balanced;
    }
    
}