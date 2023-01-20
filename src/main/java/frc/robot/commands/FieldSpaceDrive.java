package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashSet;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoystickHandler;
import frc.robot.Pigeon2Handler;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class FieldSpaceDrive extends CommandBase {
    //Declare subsystem, Joystick Handler, pigeon2
    private SwerveDriveSubsystem swerveDrive;
    private JoystickHandler joystickHandler;
    private Pigeon2Handler Pigeon2Handler;
    private SimpleWidget speedRateWidget;
    private SimpleWidget turnRateWidget;

    private double fieldAngle = 0; //Angle of away from driver from zero

    public FieldSpaceDrive(SwerveDriveSubsystem subsystem, 
    JoystickHandler joystickHandler, Pigeon2Handler Pigeon2Handler) {
        //Instantiate subsystem, Joystick Handler, pigeon2
        this.swerveDrive = subsystem;
        this.joystickHandler = joystickHandler;
        this.Pigeon2Handler = Pigeon2Handler;
        this.speedRateWidget = Shuffleboard.getTab("Preferences").addPersistent("Speed Rate", 0.5)
        .withWidget(BuiltInWidgets.kNumberSlider);
        this.turnRateWidget = Shuffleboard.getTab("Preferences").addPersistent("Turn Rate", 0.5)
        .withWidget(BuiltInWidgets.kNumberSlider);
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        Pigeon2Handler.printEverything();
        joystickHandler.updateDeadZone();

        //Set speed and turn rates for full throttle and not full throttle
        double speedRate = speedRateWidget.getEntry().getDouble(0.5);
        double turnRate = turnRateWidget.getEntry().getDouble(0.5);

        // if (joystickHandler.isFullThrottle()) {
        //     speedRate = 1;
        //     turnRate = .6;
        // }

        //Set xval, yval, spinval to the scaled values from the joystick, bounded on [-1, 1]
        double xval = Math.max(Math.min(joystickHandler.getAxis0() * -speedRate, 1), -1);
        double yval = Math.max(Math.min(joystickHandler.getAxis1() * speedRate, 1), -1);
        double spinval = Math.max(Math.min(joystickHandler.getAxis5() * turnRate, 1), -1);
        
        //spinval = -Math.pow(spinval, 3);
        
        // if (xval >= 0){
        //     xval = Math.pow(xval, 2);
        // }else{
        //     xval = -Math.pow(xval, 2);
        // }
        // if (yval >= 0){
        //     yval = Math.pow(yval, 2);
        // }else{
        //     yval = -Math.pow(yval, 2);
        // }

        //xval *= -1; //Left right swap
        

        double robotAngle = Pigeon2Handler.getAngleRad() - fieldAngle;

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

    public HashSet<Subsystem> zero() { //Zeroes direction
        HashSet<Subsystem> tree = new HashSet<Subsystem>();
        tree.add(swerveDrive);
        this.fieldAngle = Pigeon2Handler.getAngleRad(); // + Math.PI
        swerveDrive.setFieldAngle(fieldAngle);
        return tree;
    }

}
