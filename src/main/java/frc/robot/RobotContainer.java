package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FieldSpaceDrive;
import frc.robot.commands.FollowLeader;
import frc.robot.commands.RobotSpaceDrive;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
    // test commit
    public SwerveDriveSubsystem swerveDriveSubsystem;
    public Pigeon2Handler pigeon;

    private FieldSpaceDrive fieldSpaceDriveCommand;
    private RobotSpaceDrive robotSpaceDriveCommand;
    private FollowLeader followLeader;
    public PathHandler pathHandler;

    private JoystickHandler joystickHandler3;
    private JoystickHandler joystickHandler4;
    //private SendableChooser<String> autoSelect;

    public SwerveDriveSubsystem getSwervedriveSubsystem() {
        return swerveDriveSubsystem;
    }

    public Pigeon2Handler getPigeon2Handler() {
        return pigeon;
    }

    public RobotContainer() {
        pigeon = new Pigeon2Handler(); // pigeon2 input
        swerveDriveSubsystem = new SwerveDriveSubsystem(pigeon);
        joystickHandler3 = new JoystickHandler(3);
        joystickHandler4 = new JoystickHandler(4);

        // Field space uses pigeon2 to get its angle
        fieldSpaceDriveCommand = new FieldSpaceDrive(swerveDriveSubsystem, joystickHandler3, pigeon);
        robotSpaceDriveCommand = new RobotSpaceDrive(swerveDriveSubsystem, joystickHandler3);
        swerveDriveSubsystem.setDefaultCommand(fieldSpaceDriveCommand);
        //swerveDriveSubsystem.setDefaultCommand(targetSpaceDriveCommand);
        followLeader = new FollowLeader(swerveDriveSubsystem);
        pathHandler = new PathHandler(swerveDriveSubsystem);


        /*autoSelect = new SendableChooser<String>();
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
        autoSelect.addOption("pigeon2", "Nav");
        autoSelect.addOption("None", "None");
            
        Shuffleboard.getTab("Preferences").add("Auto Select", autoSelect);
        //Shuffleboard.getTab("Preferences").add("Auto Turn PID", RotateToAngle.angleController);
        */
        //limeLightSubsystem.setServoAngle(35);
        //RotateToZero.setInitialAngle(navX.getAngleRad());
        //pigeon.setInitialAngle();
        //pigeon.zeroYaw();
        
        //fieldSpaceDriveCommand.zero();

        configureButtonBindings();
    }

    public Command getAutoCommandGroup() {
        return pathHandler.balanceAuto();
    }

    private void configureButtonBindings() {
        
        joystickHandler3.button(1).whileTrue(followLeader);
        // Holding 7 will enable robot space drive, instead of field space
        joystickHandler3.button(2).whileTrue(robotSpaceDriveCommand);
        // This will set the current orientation to be "forward" for field drive
        joystickHandler3.button(3).whileTrue(new InstantCommand(() -> fieldSpaceDriveCommand.zero()));
        // This will reset odometry for Swerve drive
        joystickHandler3.button(4).whileTrue(new InstantCommand(() -> swerveDriveSubsystem.resetPose()));

        joystickHandler3.button(6).whileTrue(new InstantCommand(() -> swerveDriveSubsystem.resetSensors()));

        joystickHandler4.button(2).onFalse(pathHandler.simpleAuto());

        //joystickHandler4.button(3).whileTrue(pathHandler.TeleopScore(false));
    }
}