package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoystickHandler;

import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotSpaceDrive extends CommandBase {
    //Declare subsystem and Joystick Handler
    private final SwerveDriveSubsystem swerveDrive;
    private JoystickHandler joystickHandler;

    public RobotSpaceDrive(SwerveDriveSubsystem subsystem, JoystickHandler joystickHandler) {
        //Instantiate subsystem and Joystick Handler
        this.swerveDrive = subsystem;
        this.joystickHandler = joystickHandler;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        //Set speed rate and turn rate
        double speedRate = Preferences.getDouble("SpeedRate", 0.3);
        double turnRate = Preferences.getDouble("TurnRate", 0.1);

        if (joystickHandler.isFullThrottle()) {
            speedRate = 1;
            turnRate = .6;
        }

        //Set xval, yval, spinval to the scaled values from the joystick, bounded on [-1, 1]
        double xval = Math.max(Math.min(joystickHandler.getAxis1() * speedRate, 1), -1);
        double yval = Math.max(Math.min(joystickHandler.getAxis0() * speedRate, 1), -1);
        double spinval = Math.max(Math.min(joystickHandler.getAxis5() * turnRate, 1), -1);

        swerveDrive.drive(new ChassisSpeeds(xval, yval, spinval));
    }
}
