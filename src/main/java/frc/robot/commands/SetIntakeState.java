package frc.robot.commands;

import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Implementation "inspired" by Pride of the North

public class SetIntakeState extends CommandBase {

    PneumaticSubsystem pneumatic;
    int state;


    public SetIntakeState(PneumaticSubsystem pneumaticSubsystem, int state) 
    {
        this.pneumatic = pneumaticSubsystem;
        this.state = state;
    }
    
    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute(){
        pneumatic.setPneumaticState(state);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return false;
    }

}
