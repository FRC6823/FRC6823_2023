package frc.robot;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RepeatCommand;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.FieldSpaceDrive;
import frc.robot.commands.LineUp;
//import frc.robot.PositionHandler;
import frc.robot.commands.RobotSpaceDrive;
import frc.robot.subsystems.GripperAngleSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.PulleySubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
    // test commit
    public SwerveDriveSubsystem swerveDrive;
    public Pigeon2Handler pigeon;
    public LimeLightSubsystem limeLight;
    public PneumaticSubsystem pneumatics;
    public LiftSubsystem lift;
    public PulleySubsystem pulley;
    public GripperAngleSubsystem gripperAngle;

    private FieldSpaceDrive fieldSpaceDriveCommand;
    private RobotSpaceDrive robotSpaceDriveCommand;
    private PositionHandler positionHandler;
    private PathHandler pathHandler;

    private JoystickHandler joystickHandler3;
    private JoystickHandler joystickHandler4;

    //private SendableChooser<String> autoSelect;

    public SwerveDriveSubsystem getSwervedriveSubsystem() {
        return swerveDrive;
    }

    public Pigeon2Handler getPigeon2Handler() {
        return pigeon;
    }

    public RobotContainer() {
        pigeon = new Pigeon2Handler(); // pigeon2 input
        swerveDrive = new SwerveDriveSubsystem(pigeon);
        limeLight = new LimeLightSubsystem();
        pneumatics = new PneumaticSubsystem();
        lift = new LiftSubsystem();
        pulley = new PulleySubsystem();
        gripperAngle = new GripperAngleSubsystem();


        joystickHandler3 = new JoystickHandler(3);
        joystickHandler4 = new JoystickHandler(4);

        // Field space uses pigeon2 to get its angle
        fieldSpaceDriveCommand = new FieldSpaceDrive(swerveDrive, joystickHandler3, pigeon);
        robotSpaceDriveCommand = new RobotSpaceDrive(swerveDrive, joystickHandler3);
        swerveDrive.setDefaultCommand(fieldSpaceDriveCommand);

        positionHandler = new PositionHandler(lift, pulley, gripperAngle);
        pathHandler = new PathHandler(swerveDrive);
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
        joystickHandler3.button(1).whileTrue(new InstantCommand(() -> {swerveDrive.brake(); fieldSpaceDriveCommand.drive(false);})) 
                                                .onFalse(new InstantCommand(() -> fieldSpaceDriveCommand.drive(true)));
        
        // Holding 7 will enable robot space drive, instead of field space
        joystickHandler3.button(2).whileTrue(robotSpaceDriveCommand).onFalse(fieldSpaceDriveCommand);

        // This will set the current orientation to be "forward" for field drive
        joystickHandler3.button(3).whileTrue(new InstantCommand(() -> fieldSpaceDriveCommand.zero()));

        // This will reset odometry for Swerve drive
        //joystickHandler3.button(4).whileTrue(new InstantCommand(() -> swerveDriveSubsystem.resetPose()));

        // This will reset motor positions
        //joystickHandler3.button(6).whileTrue(new InstantCommand(() -> swerveDriveSubsystem.resetSensors()));

        //Move to score +1 node
        joystickHandler3.povLeft().whileTrue(new LineUp(swerveDrive, limeLight, "left"));           

        //Move to score -1 node
        joystickHandler3.povRight().whileTrue(new LineUp(swerveDrive, limeLight, "right"));

        //Move to score center node
        joystickHandler3.povDown().whileTrue(new LineUp(swerveDrive, limeLight, "center"));

        //Pickup
        joystickHandler3.povUp().whileTrue(new LineUp(swerveDrive, limeLight, "pickup"));









        
        
        //Open gripper
        joystickHandler4.button(5).whileTrue(new InstantCommand(() -> pneumatics.setPneumaticState(1)))
                                                .onFalse(new InstantCommand(() -> pneumatics.setPneumaticState(2)));
        //Close gripper
        joystickHandler4.button(6).whileTrue(new InstantCommand(() -> pneumatics.setPneumaticState(0)))
                                                .onFalse(new InstantCommand(() -> pneumatics.setPneumaticState(2)));



        //Manual control commands
        joystickHandler4.button(2).whileTrue(new InstantCommand(() -> {lift.setSpeed(0.5); 
                                                                                    lift.setMode(false);}))
                                                .onFalse(new InstantCommand(() -> {lift.setSpeed(0); 
                                                                                    lift.setMode(true);}));

        joystickHandler4.button(3).whileTrue(new InstantCommand(() -> {lift.setSpeed(-0.5); 
                                                                                    lift.setMode(false);}))
                                                .onFalse(new InstantCommand(() -> {lift.setSpeed(0); 
                                                                                    lift.setMode(true);}));

        joystickHandler4.button(4).whileTrue(new InstantCommand(() -> {pulley.setSpeed(1); 
                                                                                    pulley.setMode(false);}))
                                                .onFalse(new InstantCommand(() -> {pulley.setSpeed(0); 
                                                                                    pulley.setMode(true);}));

        joystickHandler4.button(1).whileTrue(new InstantCommand(() -> {pulley.setSpeed(-1); 
                                                                                        pulley.setMode(false);}))
                                                    .onFalse(new InstantCommand(() -> {pulley.setSpeed(0); 
                                                                                        pulley.setMode(true);}));

        joystickHandler4.povLeft().whileTrue(new InstantCommand(() -> gripperAngle.setSetPoint(1))).onFalse(new InstantCommand(() -> gripperAngle.setSetPoint(0)));

        joystickHandler4.povRight().whileTrue(new InstantCommand(() -> gripperAngle.setSetPoint(-1))).onFalse(new InstantCommand(() -> gripperAngle.setSetPoint(0)));



        //Records current position of lift/arm system
        joystickHandler4.button(10).whileTrue(new InstantCommand(() -> {positionHandler.capturePose();}));

        //Cycling through presets
        joystickHandler4.povUp().whileTrue(new InstantCommand(() -> {positionHandler.increaseIndex();})).whileFalse(new InstantCommand(() -> positionHandler.setPose()));

        joystickHandler4.povDown().whileTrue(new InstantCommand(() -> {positionHandler.decreaseIndex();})).whileFalse(new InstantCommand(() -> positionHandler.setPose()));
    }
}