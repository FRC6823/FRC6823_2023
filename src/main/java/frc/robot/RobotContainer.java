package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.FieldSpaceDrive;
import frc.robot.commands.PositionHandler;
import frc.robot.commands.RobotSpaceDrive;
import frc.robot.subsystems.GripperAngleSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.PulleySubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
    // test commit
    public SwerveDriveSubsystem swerveDriveSubsystem;
    public Pigeon2Handler pigeon;
    public PneumaticSubsystem pneumaticSubsystem;
    public LiftSubsystem liftSubsystem;
    public PulleySubsystem pulleySubsystem;
    public GripperAngleSubsystem gripperAngleSubsystem;

    private FieldSpaceDrive fieldSpaceDriveCommand;
    private RobotSpaceDrive robotSpaceDriveCommand;
    private PositionHandler positionHandler;
    private PathHandler pathHandler;

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
        pneumaticSubsystem = new PneumaticSubsystem();
        liftSubsystem = new LiftSubsystem();
        pulleySubsystem = new PulleySubsystem();
        gripperAngleSubsystem = new GripperAngleSubsystem();


        joystickHandler3 = new JoystickHandler(3);
        joystickHandler4 = new JoystickHandler(4);

        // Field space uses pigeon2 to get its angle
        fieldSpaceDriveCommand = new FieldSpaceDrive(swerveDriveSubsystem, joystickHandler3, pigeon);
        robotSpaceDriveCommand = new RobotSpaceDrive(swerveDriveSubsystem, joystickHandler3);
        swerveDriveSubsystem.setDefaultCommand(fieldSpaceDriveCommand);

        positionHandler = new PositionHandler(liftSubsystem, pulleySubsystem, gripperAngleSubsystem);
        pathHandler = new PathHandler(swerveDriveSubsystem);
        //liftSubsystem.setDefaultCommand(positionHandler);
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

        //Snow plow break
        joystickHandler3.button(1)

        .whileTrue(new InstantCommand(() -> {swerveDriveSubsystem.brake();
                                            fieldSpaceDriveCommand.drive(false);})) 

        .onFalse(new InstantCommand(() -> fieldSpaceDriveCommand.drive(true)));
        
        // Holding 7 will enable robot space drive, instead of field space
        //joystickHandler3.button(2).whileTrue(robotSpaceDriveCommand).onFalse(fieldSpaceDriveCommand);

        // This will set the current orientation to be "forward" for field drive
        joystickHandler3.button(3).whileTrue(new InstantCommand(() -> fieldSpaceDriveCommand.zero()));

        // This will reset odometry for Swerve drive
        joystickHandler3.button(4).whileTrue(new InstantCommand(() -> swerveDriveSubsystem.resetPose()));

        // This will reset motor positions
        joystickHandler3.button(6).whileTrue(new InstantCommand(() -> swerveDriveSubsystem.resetSensors()));

        //Move to score -1 node
        //joystickHandler3.povLeft().whileTrue(new InstantCommand(() -> swerveDriveSubsystem.brake()))
                                    //.onFalse(pathHandler.scoreLeft());                      

        //Move to score +1 node
        //joystickHandler3.povRight().whileTrue(new InstantCommand(() -> swerveDriveSubsystem.brake()))
                                    //.onFalse(pathHandler.scoreLeft());









        
        
        //Close gripper
        joystickHandler4.button(5).whileTrue(new InstantCommand(() -> pneumaticSubsystem.setPneumaticState(1)))
                                                .onFalse(new InstantCommand(() -> pneumaticSubsystem.setPneumaticState(2)));
        
        //Close gripper
        joystickHandler4.button(6).whileTrue(new InstantCommand(() -> pneumaticSubsystem.setPneumaticState(0)))
                                                .onFalse(new InstantCommand(() -> pneumaticSubsystem.setPneumaticState(2)));

        joystickHandler4.button(2).whileTrue(new InstantCommand(() -> liftSubsystem.minusSetPoint()));
        joystickHandler4.button(3).whileTrue(new InstantCommand(() -> liftSubsystem.plusSetPoint()));
        //Driver control lift/arm state
        //joystickHandler4.button(5)
        
        //.whileTrue(new InstantCommand(() -> {
                                            
                                            
                                            //liftSubsystem.setSetPoint(joystickHandler4.getAxis1());}))    

        //.onFalse(new InstantCommand(() -> {
                                            //pulleySubsystem.setSetPoint(0);
                                            
                                            //liftSubsystem.setSetPoint(0);}));

        joystickHandler4.button(1).whileTrue(new InstantCommand(() -> pulleySubsystem.minusSetPoint(-joystickHandler4.getAxis2())));
        joystickHandler4.button(4).whileTrue(new InstantCommand(() -> pulleySubsystem.plusSetPoint(joystickHandler4.getAxis3())));
        //Records current position of lift/arm system
        //joystickHandler4.button(6)

        //.whileTrue(new InstantCommand(() -> {positionHandler.setState(true); 
                                            //positionHandler.capturePose();}))

        //.onFalse(new InstantCommand(() -> positionHandler.setState(false)));
    }
}