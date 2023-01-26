package frc.robot;


//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.RunCommand;
//import frc.robot.commands.FieldSpaceDrive;
//import frc.robot.commands.RobotSpaceDrive;
import frc.robot.commands.SetIntakeState;
import frc.robot.commands.CompressorOff;
import frc.robot.subsystems.PneumaticSubsystem;

//import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
    // test commit
    //public SwerveDriveSubsystem swerveDriveSubsystem;
    final public PneumaticSubsystem pneumaticSubsystem;
    //public Pigeon2Handler pigeon;

    //private FieldSpaceDrive fieldSpaceDriveCommand;
    //private RobotSpaceDrive robotSpaceDriveCommand;

    //private JoystickHandler joystickHandler3;
    private JoystickHandler joystickHandler4;
    public SetIntakeState setIntakeStateOpen;
    public SetIntakeState setIntakeStateClosed;
    public SetIntakeState hold;
    public CompressorOff stopCompressor;
    //private SendableChooser<String> autoSelect;

    //public SwerveDriveSubsystem getSwervedriveSubsystem() {
        //return swerveDriveSubsystem;
    //}

    //public Pigeon2Handler getPigeon2Handler() {
        //return pigeon;
    //}

    public PneumaticSubsystem getPneumaticSubsystem() {
        return pneumaticSubsystem;
    }

    public RobotContainer() {
        //swerveDriveSubsystem = new SwerveDriveSubsystem();
        pneumaticSubsystem = new PneumaticSubsystem();
        //joystickHandler3 = new JoystickHandler(3);
        joystickHandler4 = new JoystickHandler(4);
        setIntakeStateOpen = new SetIntakeState(pneumaticSubsystem, 1);
        setIntakeStateClosed = new SetIntakeState(pneumaticSubsystem, 0);
        hold = new SetIntakeState(pneumaticSubsystem, 2);
        stopCompressor = new CompressorOff(pneumaticSubsystem);
        //pigeon = new Pigeon2Handler(); // pigeon2 input

        // Field space uses pigeon2 to get its angle
        //fieldSpaceDriveCommand = new FieldSpaceDrive(swerveDriveSubsystem, joystickHandler3, pigeon);
        //robotSpaceDriveCommand = new RobotSpaceDrive(swerveDriveSubsystem, joystickHandler3);
        //swerveDriveSubsystem.setDefaultCommand(fieldSpaceDriveCommand);
        //swerveDriveSubsystem.setDefaultCommand(targetSpaceDriveCommand);

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
        //fieldSpaceDriveCommand.zero();


        configureButtonBindings();
    }

    /*public AutoCommandGroup getAutoCommandGroup() {
        if (autoSelect.getSelected() != null)
            auton = new AutoCommandGroup(this, autoSelect.getSelected());
        else
            auton = new AutoCommandGroup(this, "None");
        return auton;
    }*/

    private void configureButtonBindings() {
        //RotateToAngle.setInitialAngle(navX.getAngleRad());
        //RotateToZero.setInitialAngle(navX.getAngleRad());

        // Toggles intake state
        joystickHandler4.button(3).whileTrue(setIntakeStateOpen);
        joystickHandler4.button(2).whileTrue(setIntakeStateClosed);
        joystickHandler4.button(1).whileTrue(hold);
        joystickHandler4.button(4).whileTrue(stopCompressor);
        // This will set the current orientation to be "forward" for field drive
        //joystickHandler3.button(3).onTrue(fieldSpaceDriveCommand);

        // Holding 7 will enable robot space drive, instead of field space
        //joystickHandler3.button(2).whileTrue(robotSpaceDriveCommand);
    }
}