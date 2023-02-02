package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ResetOdometry extends CommandBase{

    private SwerveDriveSubsystem swerveDrive;

    public ResetOdometry(SwerveDriveSubsystem subsystem)
    {
        swerveDrive = subsystem;
    }

    @Override
    public void execute()
    {
        swerveDrive.resetPose();
    }
}
