
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        yawPid = new PIDController(0.2, 0, 0);
    }

    public void initialize(){
        pid.setSetpoint(0);
        yawPid.setSetpoint(180);
    }

    @Override
    public void execute(){
        if (MathUtil.clipToZero(pigeon2.getRoll(), 1) != 0){
            swerveDriveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(MathUtil.getSign(pid.calculate(pigeon2.getPitch())) * -0.5, 0, yawPid.calculate(pigeon2.getPositiveYaw().getDegrees()), pigeon2.getAngleDeg()));
        }
        else{
            swerveDriveSubsystem.drive(new ChassisSpeeds(0,0,0));
        }
    }

    public boolean isFinished(){
        return false;
    }
    
}