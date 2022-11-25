package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoAim2d extends CommandBase {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private LimeLightSubsystem limeLightSubsystem;
    private boolean isFinished = false;
    private final double MARGIN = 5; // margin of degrees
    private final double P = 1;
    private final double I = 0.01;
    private double tX;
    private int pipeline;
    private PIDController pid;

    public AutoAim2d(SwerveDriveSubsystem swerveDriveSubsystem, LimeLightSubsystem limeLightSubsystem,
            int pipeline) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.limeLightSubsystem = limeLightSubsystem;
        addRequirements(swerveDriveSubsystem, limeLightSubsystem);
        this.pipeline = pipeline;
        tX = 0;
        pid = new PIDController(P, I, 0);
    }

    @Override
    public void execute() {
        if (!limeLightSubsystem.hasTarget() && tX == 0) {
            double rotateCommand = -0.2;
            swerveDriveSubsystem.drive(0, 0, rotateCommand);
        }

        if (limeLightSubsystem.hasTarget()) {
            double currentAngle = limeLightSubsystem.getTxRad();
            if (currentAngle != 0){
                tX = currentAngle / Math.PI;
            }
            swerveDriveSubsystem.drive(0, 0, pid.calculate(4 * tX, 0));
            if (Math.abs(limeLightSubsystem.getTx()) < MARGIN) {
                isFinished = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void initialize() {
        limeLightSubsystem.setPipeline(pipeline);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }
}