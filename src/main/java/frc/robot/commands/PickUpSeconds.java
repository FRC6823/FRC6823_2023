package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class PickUpSeconds extends CommandBase {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private LimeLightSubsystem limeLightSubsystem;

    private PIDController aimController;
    private Timer timer;

    private int pipeline;
    private double seconds;
    private double power;
    private boolean isItFinished;

    public PickUpSeconds(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem,
            LimeLightSubsystem limeLightSubsystem, int pipeline, double seconds) {

        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.limeLightSubsystem = limeLightSubsystem;

        this.timer = new Timer();

        this.seconds = seconds;
        this.pipeline = pipeline;
        power = 0.3;
        isItFinished = false;

        addRequirements(swerveDriveSubsystem, intakeSubsystem, limeLightSubsystem);
    }

    public PickUpSeconds(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem,
            LimeLightSubsystem limeLightSubsystem, int pipeline, double power, double seconds) {

        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.limeLightSubsystem = limeLightSubsystem;

        this.timer = new Timer();

        this.seconds = seconds;
        this.pipeline = pipeline;
        this.power = power;
        isItFinished = false;

        addRequirements(swerveDriveSubsystem, intakeSubsystem, limeLightSubsystem);
    }

    public PickUpSeconds(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem, 
        double power, double seconds) {

        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        this.timer = new Timer();

        this.seconds = seconds;
        this.power = power;
        isItFinished = false;

        addRequirements(swerveDriveSubsystem, intakeSubsystem);
    }

    @Override
    public void execute() {
        if (limeLightSubsystem != null){
            double aimCommand = aimController.calculate(limeLightSubsystem.getTx());

            limeLightSubsystem.setPipeline(pipeline);

            swerveDriveSubsystem.drive(0, -power, aimCommand * -1);
        }else{
            swerveDriveSubsystem.drive(0, -power, 0);
        }

        intakeSubsystem.intake();

        if (timer.hasElapsed(seconds)){
            isItFinished = true;
        }
    }

    @Override
    public void initialize() {
        aimController = new PIDController(.016, 0, 0);
        aimController.setSetpoint(0);
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return isItFinished;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
        intakeSubsystem.stopIntake();
        isItFinished = false;
    }
}