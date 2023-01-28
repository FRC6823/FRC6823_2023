package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroFieldSpace extends CommandBase{
    
    private FieldSpaceDrive fieldSpaceDrive;

    public ZeroFieldSpace (FieldSpaceDrive field)
    {
        fieldSpaceDrive = field;
    }

    @Override
    public void execute()
    {
        fieldSpaceDrive.zero();
    }
}
