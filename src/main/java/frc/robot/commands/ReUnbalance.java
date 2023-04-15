package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pigeon2Handler;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.MathUtil;

public class ReUnbalance extends CommandBase{
    private boolean balanced;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private Pigeon2Handler pigeon2;
    private PIDController pid, yawPid;

    public ReUnbalance(Pigeon2Handler pigeon2, SwerveDriveSubsystem swerveDriveSubsystem){
        addRequirements(swerveDriveSubsystem);
        this.pigeon2 = pigeon2;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        pid = new PIDController(.3, 0, 0);
        yawPid = new PIDController(Constants.yawKp, Constants.yawKi, 0);
        yawPid.enableContinuousInput(0, 360);
    }

    public void initialize(){
        balanced = false;
        pid.setSetpoint(-10.5);
        yawPid.setSetpoint(180);
    }

    @Override
    public void execute(){
        if (MathUtil.clipToZero(pigeon2.getRoll() + 11, 1) == 0){
            balanced = true;
        }
        else if (!balanced){
            swerveDriveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-2, 0, yawPid.calculate(pigeon2.getYaw()), pigeon2.getAngleDeg()));
        }
    }

    public boolean isFinished(){
        return balanced;
    }
    
}