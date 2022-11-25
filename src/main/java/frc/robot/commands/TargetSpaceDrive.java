package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoystickHandler;
import frc.robot.NavXHandler;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TargetSpaceDrive extends CommandBase {
    //Declare subsystem, Joystick Handler, NavX
    private SwerveDriveSubsystem swerveDrive;
    private JoystickHandler joystickHandler;
    private NavXHandler navXHandler;
    private LimeLightSubsystem limelight;

    private double fieldAngle; //Angle of away from driver from zero
    private double tX;

    public TargetSpaceDrive(SwerveDriveSubsystem subsystem, 
    JoystickHandler joystickHandler, LimeLightSubsystem limelight, 
    NavXHandler navXHandler) {
        //Instantiate subsystem, Joystick Handler, NavX
        this.swerveDrive = subsystem;
        this.joystickHandler = joystickHandler;
        this.navXHandler = navXHandler;
        this.limelight = limelight;

        fieldAngle = 0;
        tX = 0;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        navXHandler.printEverything();
        if (limelight.getTxRad() != 0){
            tX = limelight.getTxRad() / Math.PI;
        }

        //Set speed and turn rates for full throttle and not full throttle
        if (!Preferences.containsKey("Speed Rate") || Preferences.getDouble("Speed Rate", -2) == -2)
            Preferences.setDouble("Speed Rate", 0.5);
        double speedRate = Preferences.getDouble("Speed Rate", 0.5);

        // if (joystickHandler.isFullThrottle()) {
        //     speedRate = 1;
        //     turnRate = .6;
        // }

        //Set xval, yval, spinval to the scaled values from the joystick, bounded on [-1, 1]
        double xval = Math.max(Math.min(joystickHandler.getAxis0() * speedRate, 1), -1);
        double yval = Math.max(Math.min(joystickHandler.getAxis1() * - speedRate, 1), -1);
        double spinval = -tX; //Raise to a power?

        //xval *= -1; //Left right swap
        

        double robotAngle = navXHandler.getAngleRad() - fieldAngle;

        // mapping field space to robot space
        double txval = getTransX(xval, yval, robotAngle);
        double tyval = getTransY(xval, yval, robotAngle);

        swerveDrive.drive(txval, tyval, spinval);
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
}
