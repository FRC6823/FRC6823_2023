package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class FullHalt extends CommandBase {
    private SwerveDriveSubsystem swerveDrive;
    private ShooterSubsystem shooter;
    private ConveyorSubsystem conveyor;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;
    private boolean isFinished;

    public FullHalt(SwerveDriveSubsystem swerveDrive, ShooterSubsystem shooter, ConveyorSubsystem conveyor, IntakeSubsystem intake, LiftSubsystem lift) {
        //Instantiate subsystem, Joystick Handler
        this.swerveDrive = swerveDrive;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.intake = intake;
        this.lift = lift;
        isFinished = false;

        addRequirements(this.shooter);
    }

    public FullHalt(SwerveDriveSubsystem swerveDrive, ShooterSubsystem shooter, ConveyorSubsystem conveyor, IntakeSubsystem intake) {
        //Instantiate subsystem, Joystick Handler
        this.swerveDrive = swerveDrive;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.intake = intake;
        isFinished = false;

        addRequirements(this.shooter);
    }

    @Override
    public void execute() {
        swerveDrive.stop();
        shooter.loadStop();
        shooter.shootStop();
        conveyor.stopConvey();
        intake.stopAngle();
        intake.stopIntake();
        if(lift != null)
            lift.liftStop();
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
