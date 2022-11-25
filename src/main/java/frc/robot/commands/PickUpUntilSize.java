package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class PickUpUntilSize extends CommandBase {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private LimeLightSubsystem limeLightSubsystem;

    private PIDController aimController;
    private PIDController distController;

    private int pipeline;
    private boolean isItFinished;
    private final double MARGIN = 5;

    public PickUpUntilSize(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem,
            LimeLightSubsystem limeLightSubsystem, int pipeline) {

        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.limeLightSubsystem = limeLightSubsystem;

        this.pipeline = pipeline;
        isItFinished = false;

        addRequirements(swerveDriveSubsystem, intakeSubsystem, limeLightSubsystem);
    }

    @Override
    public void execute() {
        double aimCommand = aimController.calculate(limeLightSubsystem.getTx());
        double distCommand = distController.calculate(limeLightSubsystem.getTy());

        limeLightSubsystem.setPipeline(pipeline);

        swerveDriveSubsystem.drive(0, -distCommand, aimCommand * -1);

        intakeSubsystem.intake();

        if (Math.abs(limeLightSubsystem.getTy()) < MARGIN){
            isItFinished = true;
        }
    }

    @Override
    public void initialize() {
        aimController = new PIDController(.016, 0, 0);
        aimController.setSetpoint(0);
        distController = new PIDController(.016, 0, 0);
        distController.setSetpoint(0);
    }

    @Override
    public boolean isFinished() {
        return isItFinished;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
        isItFinished = false;
    }
}