package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NavXHandler;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.MathUtil;

public class RotateToAngle extends CommandBase {
    //Declare subsystem, NavX, PID Controller, and necessary variables
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private NavXHandler navXHandler;
    private boolean isFinished = false;
    private final double MARGIN = 0.03; // margin in radians
    private PIDController angleController;

    private double angle;
    private static double initialDegrees;

    public RotateToAngle(SwerveDriveSubsystem swerveDriveSubsystem, NavXHandler navXHandler, double angle) {
        //Instantiate subsystem, NavX, and angle
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.navXHandler = navXHandler;
        this.angle = MathUtil.mod(angle, 2 * Math.PI);
        addRequirements(swerveDriveSubsystem);

    }

    public static void setInitialAngle(double angle) {
        RotateToAngle.initialDegrees = MathUtil.mod(angle, 2 * Math.PI);
    }

    @Override
    public void execute() {
        //Get angle and power proportional to angle left to travel
        double currentAngle = MathUtil.mod(navXHandler.getAngleRad(), 2 * Math.PI);
        double rotateCommand = angleController.calculate(currentAngle);

        //Bound power on {-0.4, 0,4]
        if (rotateCommand > 0.4) {
            rotateCommand = 0.4;
        } else if (rotateCommand < -0.4) {
            rotateCommand = -0.4;
        }

        //Print rotate power and rotate at that power
        //SmartDashboard.putNumber("ROTATE", rotateCommand);
        swerveDriveSubsystem.drive(0, 0, -rotateCommand);

        SmartDashboard.putNumber("NavX Angle", currentAngle);

        //If within margin of error, set isFinished to true
        if (Math.abs((currentAngle - (initialDegrees + angle)) % (2 * Math.PI)) < MARGIN) {
            isFinished = true;
        }

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void initialize() {
        angleController = new PIDController(.35, .001, 0);
        angleController.enableContinuousInput(0, Math.PI * 2);
        angleController.setSetpoint(MathUtil.mod(initialDegrees + angle, 2 * Math.PI));
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }
}
