package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class GoBackwards extends CommandBase {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private boolean isFinished;
    private double percent;
    private double seconds;
    private Timer timer;

    public GoBackwards(SwerveDriveSubsystem swerveDriveSubsystem, double percentPower, double seconds) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        addRequirements(swerveDriveSubsystem);
        timer = new Timer();
        isFinished = false;
        this.percent = percentPower;
        this.seconds = seconds;
    }

    @Override
    public void execute() {
        swerveDriveSubsystem.drive(0, percent, 0);

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
        swerveDriveSubsystem.stop();
    }
}
