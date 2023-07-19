package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Pigeon2Handler;
import frc.robot.PositionHandler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoCommandGroup extends SequentialCommandGroup{
    private PneumaticSubsystem pneumatic;
    private SwerveDriveSubsystem swerve;
    private Pigeon2Handler pigeon;
    private PositionHandler positionHandler;

    public AutoCommandGroup(RobotContainer container, int auto){
        pneumatic = container.getPneumatics();
        swerve = container.getSwervedriveSubsystem();
        pigeon = container.getPigeon2Handler();
        positionHandler = container.getPositionHandler();
    }
}
