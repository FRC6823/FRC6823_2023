package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BrakeMode extends CommandBase {
    private SwerveDriveSubsystem swerveDrive;
    private boolean isFinished;
    private double seconds;
    private Timer timer;

    public BrakeMode(SwerveDriveSubsystem swerveDrive, double seconds) {
        //Instantiate subsystem, Joystick Handler
        this.swerveDrive = swerveDrive;
        isFinished = false;
        timer = new Timer();
        this.seconds = seconds;
    }

    @Override
    public void execute() {
        swerveDrive.brake();
        if (timer.hasElapsed(seconds)) {
            isFinished = true;
        }
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean inturrupted) {
        isFinished = false;
        swerveDrive.coast();
    }
}
