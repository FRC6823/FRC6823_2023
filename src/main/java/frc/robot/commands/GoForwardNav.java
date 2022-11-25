package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NavXHandler;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

public class GoForwardNav extends CommandBase {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private NavXHandler navX;
    private PIDController pid;
    private boolean isFinished;
    private double angle;
    private double turn;
    private double percent;
    private double seconds;
    private Timer timer;

    public GoForwardNav(SwerveDriveSubsystem swerveDriveSubsystem, NavXHandler navX, double percentPower, double seconds) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.navX = navX;
        addRequirements(swerveDriveSubsystem);
        timer = new Timer();
        isFinished = false;
        this.percent = percentPower;
        this.seconds = seconds;
    }

    @Override
    public void execute() {
        turn = pid.calculate(MathUtil.mod(navX.getAngleRad(), 2 * Math.PI));
        swerveDriveSubsystem.drive(0, percent, turn);

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
        pid = new PIDController(.4, 0, 0);
        pid.enableContinuousInput(0, Math.PI * 2);
        angle = MathUtil.mod(navX.getAngleRad(), 2 * Math.PI);
        pid.setSetpoint(angle);
    }

    @Override
    public void end(boolean interrupted) {
        isFinished = false;
        swerveDriveSubsystem.stop();
    }
}
