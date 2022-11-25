package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class HammerDrop extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private boolean isFinished;
    private double seconds;
    private Timer timer;

    public HammerDrop(IntakeSubsystem intakeSubsystem, double seconds) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
        timer = new Timer();
        isFinished = false;
        this.seconds = seconds;
    }

    @Override
    public void execute() {
        intakeSubsystem.backAngle(0.5);

        if (timer.hasElapsed(seconds)) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        isFinished = false;
        intakeSubsystem.stopAngle();
    }
}