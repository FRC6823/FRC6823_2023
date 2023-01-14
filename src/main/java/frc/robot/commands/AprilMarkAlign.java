package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NavXHandler;
import frc.robot.subsystems.PhotonCameraSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AprilMarkAlign extends CommandBase {
    //Declare subsystem, Joystick Handler, NavX
    private SwerveDriveSubsystem swerveDrive;
    private NavXHandler navXHandler;
    private PhotonCameraSubsystem camera;

    private double fieldAngle; //Angle of away from driver from zero
    private double tX;
    private double skew;
    private boolean isFinished;

    public AprilMarkAlign(SwerveDriveSubsystem subsystem, PhotonCameraSubsystem camera, 
    NavXHandler navXHandler) {
        //Instantiate subsystem, NavX
        this.swerveDrive = subsystem;
        this.navXHandler = navXHandler;
        this.camera = camera;

        fieldAngle = 0;
        tX = 0;
        skew = 0;
        isFinished = false;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        navXHandler.printEverything();
        if (camera.getTxRad() != 0){
            tX = camera.getTxRad() / Math.PI;
        }

        if (camera.getSkewRad() != 0){
            skew = camera.getSkewRad() / Math.PI;
        }
        
        double spinval = -tX;
        
        double robotAngle = navXHandler.getAngleRad() - fieldAngle;

        // mapping field space to robot space
        double txval = getTransX(0, 0, robotAngle);
        double tyval = getTransY(0, 0, robotAngle);

        swerveDrive.drive((txval + skew) / 2, (tyval + skew) / 2, spinval);

        if (Math.abs(skew) < 0.1 && Math.abs(tX) < 0.1){
            isFinished = true;
        }
    }

    private double getTransX(double x, double y, double angle) { //Returns x direction robot needs to move in
        return x * Math.cos(angle) + -y * Math.sin(angle);
    }

    private double getTransY(double x, double y, double angle) { //Returns y direction robot needs to move in
        return x * Math.sin(angle) + y * Math.cos(angle);
    }

    public void zero() { //Zeroes direction
        this.fieldAngle = navXHandler.getAngleRad() + Math.PI;
        swerveDrive.setFieldAngle(fieldAngle);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }
} 