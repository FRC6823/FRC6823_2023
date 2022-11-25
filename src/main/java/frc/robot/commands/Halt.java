package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class Halt extends CommandBase {
    private SwerveDriveSubsystem swerveDrive;
    private ShooterSubsystem shooter;
    private ConveyorSubsystem conveyor;
    private boolean isFinished;

    public Halt(SwerveDriveSubsystem swerveDrive, ShooterSubsystem shooter, ConveyorSubsystem conveyor) {
        //Instantiate subsystem, Joystick Handler
        this.swerveDrive = swerveDrive;
        this.shooter = shooter;
        this.conveyor = conveyor;
        isFinished = false;

        addRequirements(this.shooter);
    }

    @Override
    public void execute() {
        swerveDrive.stop();
        shooter.loadStop();
        shooter.shootStop();
        conveyor.stopConvey();
        isFinished = true;
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean inturrupted) {
        isFinished = false;
    }
}
