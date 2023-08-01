package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathHandler;
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
    private PathHandler pathHandler;

    public AutoCommandGroup(RobotContainer container, int auto){
        pneumatic = container.getPneumatics();
        swerve = container.getSwervedriveSubsystem();
        pigeon = container.getPigeon2Handler();
        positionHandler = container.getPositionHandler();
        pathHandler = container.getPathHandler();


        if (auto == 1){
            addCommands(new InstantCommand(() -> positionHandler.setPose(5)), new WaitCommand(0.5));
            addCommands(new InstantCommand(() -> positionHandler.setPose(4)), new WaitCommand(2.5), new InstantCommand(() -> pneumatic.togglePneumaticState()));
            addCommands(new WaitCommand(0.2), new InstantCommand(() -> positionHandler.setPose(2)), new WaitCommand(0.5));
        }
        else if (auto == 2) {
            addCommands(new InstantCommand(() -> positionHandler.setPose(5)), new WaitCommand(0.5));
            addCommands(new InstantCommand(() -> positionHandler.setPose(4)), new WaitCommand(2.5), new InstantCommand(() -> pneumatic.togglePneumaticState()));
            addCommands(new WaitCommand(0.2), new InstantCommand(() -> positionHandler.setPose(2)), new WaitCommand(0.5));
            addCommands(new WaitCommand(1), new Reverse(swerve, pigeon, 2, 2.5));
            addCommands(new WaitCommand(15)); 
        }
        else if (auto == 3) {
            addCommands(new InstantCommand(() -> positionHandler.setPose(5)), new WaitCommand(0.5));
            addCommands(new InstantCommand(() -> positionHandler.setPose(4)), new WaitCommand(2.5), new InstantCommand(() -> pneumatic.togglePneumaticState()));
            addCommands(new WaitCommand(0.2), new InstantCommand(() -> positionHandler.setPose(2)), new WaitCommand(0.5));
            addCommands(new WaitCommand(0.5), new Unbalance(pigeon, swerve), new Reverse(swerve, pigeon, 0.7, 2), new Rebalance(pigeon, swerve));
            addCommands(new WaitCommand(15)); 
        }
        else if (auto == 4) {
            addCommands(new InstantCommand(() -> positionHandler.setPose(5)), new WaitCommand(0.5));
            addCommands(new InstantCommand(() -> positionHandler.setPose(4)), new WaitCommand(2.5), new InstantCommand(() -> pneumatic.togglePneumaticState()));
            addCommands(new WaitCommand(0.2), new InstantCommand(() -> positionHandler.setPose(2)), new WaitCommand(0.5));
            addCommands(new WaitCommand(1), new Reverse(swerve, pigeon, 2, 2.5));
            addCommands(new WaitCommand(1), new InstantCommand(() -> positionHandler.setPose(0)));
            addCommands(new WaitCommand(15)); 
        }
        else if (auto == 5) {
            addCommands(new InstantCommand(() -> positionHandler.setPose(5)), new WaitCommand(0.5));
            addCommands(new InstantCommand(() -> positionHandler.setPose(4)), new WaitCommand(2.5), new InstantCommand(() -> pneumatic.togglePneumaticState()));
            addCommands(new WaitCommand(0.2), new InstantCommand(() -> positionHandler.setPose(2)), new WaitCommand(0.5));
            addCommands(new WaitCommand(1), new Reverse(swerve, pigeon, 2, 2.5));
            addCommands(new WaitCommand(1), new InstantCommand(() -> positionHandler.setPose(5)));
            addCommands(new WaitCommand(15)); 
        }
        else if (auto == 6) {
            addCommands(new InstantCommand(() -> positionHandler.setPose(5)), new WaitCommand(0.5));
            addCommands(new InstantCommand(() -> positionHandler.setPose(4)), new WaitCommand(2.5), new InstantCommand(() -> pneumatic.togglePneumaticState()));
            addCommands(new WaitCommand(0.2), new InstantCommand(() -> positionHandler.setPose(5)));
            addCommands(new WaitCommand(15)); 
        }
        else if (auto == 7) {
            addCommands(new InstantCommand(() -> positionHandler.setPose(5)), new WaitCommand(0.1));
            addCommands(new InstantCommand(() -> positionHandler.setPose(4)), new WaitCommand(2), new InstantCommand(() -> pneumatic.togglePneumaticState()));
            addCommands(new WaitCommand(0.2), new InstantCommand(() -> positionHandler.setPose(2)), new WaitCommand(0.5));
            addCommands(new Unbalance(pigeon, swerve), new Over(pigeon, swerve), new Reverse(swerve, pigeon, 1, 1.5), new ReUnbalance(pigeon, swerve), new Reverse(swerve, pigeon, 0.8, -2), new Rebalance(pigeon, swerve));
        }
        else {
            addCommands(pathHandler.balanceAuto());
        }
    }
}
