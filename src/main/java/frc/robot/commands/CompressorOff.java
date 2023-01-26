package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticSubsystem;

public class CompressorOff extends CommandBase{
    PneumaticSubsystem pneumatic;


    public CompressorOff(PneumaticSubsystem pneumaticSubsystem) 
    {
        this.pneumatic = pneumaticSubsystem;
    }
    
    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute(){
        pneumatic.setCompressorState(false);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return false;
    }

}
