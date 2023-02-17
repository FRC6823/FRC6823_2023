package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pigeon2Handler;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.MathUtil;

public class Balance extends CommandBase{
    private boolean balanced;
    private boolean accel;
    private double pitch;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private Pigeon2Handler pigeon2;

    public Balance(Pigeon2Handler pigeon2, SwerveDriveSubsystem swerveDriveSubsystem){
        this.pigeon2 = pigeon2;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        balanced = MathUtil.clipToZero(pigeon2.getPitch(), 2.15) == 0;
        pitch = pigeon2.getPitch();
        accel = false;
    }

    @Override
    public void execute(){
        while (!balanced){
            
        }
    }
    
}
