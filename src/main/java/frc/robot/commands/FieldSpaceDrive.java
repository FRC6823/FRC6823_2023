package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

//import edu.wpi.first.wpilibj2.command.Subsystem;
//import java.util.HashSet;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoystickHandler;
import frc.robot.NavXHandler;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class FieldSpaceDrive extends CommandBase {
    //Declare subsystem, Joystick Handler, navX2
    private SwerveDriveSubsystem swerveDrive;
    private JoystickHandler joystickHandler;
    private NavXHandler navX;
    private SimpleWidget speedRateWidget;
    private SimpleWidget turnRateWidget;

    public FieldSpaceDrive(SwerveDriveSubsystem subsystem, 
    JoystickHandler joystickHandler, NavXHandler navX) {
        //Instantiate subsystem, Joystick Handler, navX2
        this.swerveDrive = subsystem;
        this.joystickHandler = joystickHandler;
        this.navX = navX;
        this.speedRateWidget = Shuffleboard.getTab("Preferences").addPersistent("Speed Rate", 0.5)
        .withWidget(BuiltInWidgets.kNumberSlider);
        this.turnRateWidget = Shuffleboard.getTab("Preferences").addPersistent("Turn Rate", 0.5)
        .withWidget(BuiltInWidgets.kNumberSlider);
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        navX.printEverything();
        joystickHandler.updateDeadZone();

        //Set speed and turn rates for full throttle and not full throttle
        double speedRate = speedRateWidget.getEntry().getDouble(0.5);
        double turnRate = turnRateWidget.getEntry().getDouble(0.5);

        // if (joystickHandler.isFullThrottle()) {
        //     speedRate = 1;
        //     turnRate = .6;
        // }

        //Set xval, yval, spinval to the scaled values from the joystick, bounded on [-1, 1]
        double xval = Math.max(Math.min(joystickHandler.getAxis1() * -speedRate, 1), -1);
        double yval = Math.max(Math.min(joystickHandler.getAxis0() * -speedRate, 1), -1);
        double spinval = Math.max(Math.min(joystickHandler.getAxis5() * -turnRate, 1), -1);

        // mapping field space to robot space
        //double txval = getTransX(xval, yval, robotAngle);
        //double tyval = getTransY(xval, yval, robotAngle);

        swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xval, yval, spinval, getRobotAngle()));
    }

    public Rotation2d getRobotAngle()
    {
        return navX.getAngleDeg();
    }

    public void zero() { //Zeroes direction
        navX.zeroYaw();
    }

}
