package frc.robot.commands;

import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Implementation "inspired" by Pride of the North

public class SetIntakeState extends CommandBase {

    PneumaticSubsystem pneumatic;


    public SetIntakeState(PneumaticSubsystem pneumatic) 
    {
        this.pneumatic = pneumatic;
    }
    
    @Override
    public void initialize()
    {
        pneumatic.togglePneumaticState();
    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return false;
    }
}
