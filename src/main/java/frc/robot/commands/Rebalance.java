
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pigeon2Handler;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.MathUtil;

public class Rebalance extends CommandBase{
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private Pigeon2Handler pigeon2;
    private PIDController pid, yawPid;

    public Rebalance(Pigeon2Handler pigeon2, SwerveDriveSubsystem swerveDriveSubsystem){
        addRequirements(swerveDriveSubsystem);
        this.pigeon2 = pigeon2;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        pid = new PIDController(.1, 0, 0);
        yawPid = new PIDController(0.3, 0, 0);
    }

    public void initialize(){
        pid.setSetpoint(0);
        yawPid.setSetpoint(180);
    }

    @Override
    public void execute(){
        if (MathUtil.clipToZero(pigeon2.getRoll(), 5) != 0){
            swerveDriveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(MathUtil.getSign(pid.calculate(pigeon2.getRoll())) * -0.65, 0, yawPid.calculate(pigeon2.getYaw()), pigeon2.getAngleDeg()));
        }
        else{
            swerveDriveSubsystem.brake();
        }
    }

    public boolean isFinished(){
        return false;
    }
    
}