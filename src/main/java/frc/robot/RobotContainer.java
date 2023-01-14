package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AutoCommandGroup;
import frc.robot.commands.AutoSearchLeft;
import frc.robot.commands.AutoSearchRight;
import frc.robot.commands.FieldSpaceDrive;
import frc.robot.commands.RobotSpaceDrive;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.RotateToZero;
import frc.robot.commands.ServoTuck;
import frc.robot.commands.SwitchPipelineCommand;
import frc.robot.commands.TargetSpaceDrive;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
    // test commit
    public SwerveDriveSubsystem swerveDriveSubsystem;
    public NavXHandler navX;

    private FieldSpaceDrive fieldSpaceDriveCommand;
    private RobotSpaceDrive robotSpaceDriveCommand;
    private TargetSpaceDrive targetSpaceDriveCommand;
    private AutoCommandGroup auton;

    private JoystickHandler joystickHandler3;
    //private JoystickHandler joystickHandler4;
    public LimeLightSubsystem limeLightSubsystem;

    private SendableChooser<String> autoSelect;

    public LimeLightSubsystem getLimeLightSubsystem() {
        return limeLightSubsystem;
    }

    public SwerveDriveSubsystem getSwervedriveSubsystem() {
        return swerveDriveSubsystem;
    }

    public NavXHandler getNavXHandler() {
        return navX;
    }

    public RobotContainer() {
        swerveDriveSubsystem = new SwerveDriveSubsystem();
        joystickHandler3 = new JoystickHandler(3);
        //joystickHandler4 = new JoystickHandler(4);
        limeLightSubsystem = new LimeLightSubsystem(8);

        navX = new NavXHandler(); // navx input

        // Field space uses navx to get its angle
        fieldSpaceDriveCommand = new FieldSpaceDrive(swerveDriveSubsystem, joystickHandler3, navX);
        robotSpaceDriveCommand = new RobotSpaceDrive(swerveDriveSubsystem, joystickHandler3);
        targetSpaceDriveCommand = new TargetSpaceDrive(swerveDriveSubsystem, joystickHandler3, limeLightSubsystem, navX);
        swerveDriveSubsystem.setDefaultCommand(fieldSpaceDriveCommand);
        //swerveDriveSubsystem.setDefaultCommand(targetSpaceDriveCommand);

        autoSelect = new SendableChooser<String>();
        autoSelect.setDefaultOption("1 Ball", "1Ball"); //Look into if default not working
        autoSelect.addOption("Taxi", "Taxi");
        autoSelect.addOption("Ball 1", "Ball 1");
        autoSelect.addOption("Ball 2", "Ball 2");
        autoSelect.addOption("Ball 4", "Ball 4");
        autoSelect.addOption("Ball 7", "Ball 7");
        autoSelect.addOption("Ball 8", "Ball 8");
        autoSelect.addOption("Ball 10", "Ball 10");
        autoSelect.addOption("4 Ball Red", "4 Red");
        autoSelect.addOption("4 Ball Blue", "4 Blue");
        autoSelect.addOption("NavX", "Nav");
        autoSelect.addOption("None", "None");

        Shuffleboard.getTab("Preferences").add("Auto Select", autoSelect);
        //Shuffleboard.getTab("Preferences").add("Auto Turn PID", RotateToAngle.angleController);

        //limeLightSubsystem.setServoAngle(35);
        limeLightSubsystem.setPipeline(1); //set back to 0 for target space about tower
        RotateToZero.setInitialAngle(navX.getAngleRad());
        navX.setInitialAngle();
        fieldSpaceDriveCommand.zero();

        configureButtonBindings();
    }

    public AutoCommandGroup getAutoCommandGroup() {
        if (autoSelect.getSelected() != null)
            auton = new AutoCommandGroup(this, autoSelect.getSelected());
        else
            auton = new AutoCommandGroup(this, "None");
        return auton;
    }

    private void configureButtonBindings() {
        RotateToAngle.setInitialAngle(navX.getAngleRad());
        RotateToZero.setInitialAngle(navX.getAngleRad());
        // Hold button 8 to set the swerve just forward, this is for calibration
        // purposes
        joystickHandler3.button(8).whileTrue(() -> swerveDriveSubsystem.drive(0,
                0.1, 0));

        // This will set the current orientation to be "forward" for field drive
        joystickHandler3.button(3).onTrue(fieldSpaceDriveCommand);

        // Holding 7 will enable robot space drive, instead of field space
        joystickHandler3.button(2).whileTrue(robotSpaceDriveCommand);

        joystickHandler3.button(4).whileTrue(targetSpaceDriveCommand);

        joystickHandler3.button(13).onTrue(new 
        AutoSearchLeft(swerveDriveSubsystem, 
        limeLightSubsystem, 0));
        joystickHandler3.button(14).onTrue(new 
        AutoSearchRight(swerveDriveSubsystem, 
        limeLightSubsystem, 1));

        joystickHandler3.button(7).onTrue(new 
        ServoTuck(limeLightSubsystem));
        joystickHandler3.button(7).whileTrue(
         new SwitchPipelineCommand(limeLightSubsystem, 1));
    }
}
