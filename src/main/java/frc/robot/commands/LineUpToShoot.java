package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.LimelightTools;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

public class LineUpToShoot extends CommandBase {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private LimeLightSubsystem limeLightSubsystem;
    private boolean isFinished;
    private PIDController pid;
    private double distance;
    private double margin;
    private double power;
    private Timer timer;

    public LineUpToShoot(SwerveDriveSubsystem swerveDriveSubsystem, 
    LimeLightSubsystem limelightSubsystem, double distance) { //distance in meters
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.limeLightSubsystem = limelightSubsystem;
        addRequirements(swerveDriveSubsystem, limelightSubsystem);
        isFinished = false;
        this.distance = distance;
        margin = 0.1;
        pid = new PIDController(0.1, 0.001, 0);
    }

    @Override
    public void execute() {
        power = pid.calculate(LimelightTools.distFromTower(limeLightSubsystem.getTyRad()), distance);
        swerveDriveSubsystem.drive(0, power, 0);

        if (Math.abs(LimelightTools.distFromTower(limeLightSubsystem.getTyRad()) - distance) < margin) {
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
