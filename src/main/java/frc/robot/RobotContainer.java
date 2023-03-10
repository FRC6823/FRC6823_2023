package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.RepeatCommand;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.FieldSpaceDrive;
import frc.robot.commands.LineUp;
import frc.robot.commands.Rebalance;
import frc.robot.commands.Reverse;
//import frc.robot.PositionHandler;
import frc.robot.commands.RobotSpaceDrive;
import frc.robot.commands.Unbalance;
import frc.robot.commands.WaitUntilPose;
import frc.robot.subsystems.GripperAngleSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.PulleySubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.Constants;

public class RobotContainer {
    // test commit
    public SwerveDriveSubsystem swerveDrive;
    public Pigeon2Handler pigeon;
    public LimeLightSubsystem limeLight;
    public PneumaticSubsystem pneumatics;
    public LiftSubsystem lift;
    public PulleySubsystem pulley;
    public GripperAngleSubsystem gripperAngle;
    //public LEDSubsystem LEDs;

    private FieldSpaceDrive fieldSpaceDriveCommand;
    private RobotSpaceDrive robotSpaceDriveCommand;
    private PositionHandler positionHandler;
    private PathHandler pathHandler;
    private Unbalance unbalance;
    private Rebalance rebalance;

    private JoystickHandler joystickHandler3;
    private JoystickHandler joystickHandler4;

    private SendableChooser<Integer> startNode;
    private SendableChooser<Integer> firstPiece;
    private SendableChooser<Integer> secondNode;
    private SendableChooser<Boolean> balance;


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
        //LEDs = new LEDSubsystem(0);


        joystickHandler3 = new JoystickHandler(3);
        joystickHandler4 = new JoystickHandler(4);

        // Field space uses pigeon2 to get its angle
        fieldSpaceDriveCommand = new FieldSpaceDrive(swerveDrive, joystickHandler3, pigeon);
        robotSpaceDriveCommand = new RobotSpaceDrive(swerveDrive, joystickHandler3);
        swerveDrive.setDefaultCommand(fieldSpaceDriveCommand);

        positionHandler = new PositionHandler(lift, pulley, gripperAngle);
        pathHandler = new PathHandler(swerveDrive);
        unbalance = new Unbalance(pigeon, swerveDrive);
        rebalance = new Rebalance(pigeon, swerveDrive);
        //liftSubsystem.setDefaultCommand(positionHandler);

        startNode = new SendableChooser<Integer>();
        startNode.setDefaultOption("1", 1); //Look into if default not working
        startNode.addOption("1", 1);
        startNode.addOption("2", 2);
        startNode.addOption("3", 3);
        startNode.addOption("4", 4);
        startNode.addOption("5", 5);
        startNode.addOption("6", 6);
        startNode.addOption("7", 7);
        startNode.addOption("8", 8);
        startNode.addOption("9", 9);
    
        Shuffleboard.getTab("Preferences").add("Starting Node", startNode);



        firstPiece = new SendableChooser<Integer>();
        firstPiece.setDefaultOption("1", 1); //Look into if default not working
        //firstPiece.addOption("1", 1);
        firstPiece.addOption("2", 2);
        firstPiece.addOption("3", 3);
        firstPiece.addOption("4", 4);
    
        Shuffleboard.getTab("Preferences").add("First Piece", firstPiece);



        secondNode = new SendableChooser<Integer>();
        secondNode.setDefaultOption("1", 1); //Look into if default not working
        //secondNode.addOption("1", 1);
        secondNode.addOption("2", 2);
        secondNode.addOption("3", 3);
        secondNode.addOption("4", 4);
        secondNode.addOption("5", 5);
        secondNode.addOption("6", 6);
        secondNode.addOption("7", 7);
        secondNode.addOption("8", 8);
        secondNode.addOption("9", 9);
    
        Shuffleboard.getTab("Preferences").add("Second Node", secondNode);
        
        balance = new SendableChooser<Boolean>();
        balance.setDefaultOption("No", false); //Look into if default not working
        balance.addOption("Yes", true);
    
        Shuffleboard.getTab("Preferences").add("Balance", balance);

        pigeon.setYaw(180);
        pigeon.setPitchOffset(pigeon.getPitch());
        pigeon.setRollOffset(pigeon.getRoll());
        //fieldSpaceDriveCommand.zero();
        configureButtonBindings();
    }

    public Command getAutoCommandGroup() {
        
        SequentialCommandGroup auto = new SequentialCommandGroup(new InstantCommand(() -> positionHandler.setPose(5)), new WaitUntilPose(lift, pulley, gripperAngle));
        auto.addCommands(new InstantCommand(() -> positionHandler.setPose(4)), new WaitUntilPose(lift, pulley, gripperAngle), new WaitCommand(1), new InstantCommand(() -> pneumatics.togglePneumaticState()));
        auto.addCommands(new WaitCommand(1), new InstantCommand(() -> positionHandler.setPose(2)), new WaitUntilPose(lift, pulley, gripperAngle));
        auto.addCommands(new Reverse(swerveDrive, pigeon));
        auto.addCommands(new WaitCommand(15));
        //auto.addCommands(new Unbalance(pigeon, swerveDrive), new Rebalance(pigeon, swerveDrive));
        //return pathHandler.getPath(startNode.getSelected(), firstPiece.getSelected(), false);
                                        //new InstantCommand(() -> positionHandler.setPose(-30, 0.1)),
                                        //pathHandler.getPath(startNode.getSelected(), firstPiece.getSelected(), false));

        return auto;
    }

    private void configureButtonBindings() {

        //Snow plow break
        joystickHandler3.button(1).whileTrue(new InstantCommand(() -> pneumatics.togglePneumaticState()));
        
        // Holding 7 will enable robot space drive, instead of field space
        joystickHandler3.button(2).whileTrue(new SequentialCommandGroup(unbalance, rebalance));

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
        joystickHandler3.povUp().whileTrue(new LineUp(swerveDrive, limeLight, "center"));

        //Pickup
        joystickHandler3.button(10).whileTrue(new LineUp(swerveDrive, limeLight, "pickup"));

        //button 1 gripper toggle







        
        
        //Yellow LED
        joystickHandler4.button(5).whileTrue(new InstantCommand(() -> {pulley.disable(); lift.disable(); gripperAngle.disable();}));
        //Purple LED
        joystickHandler4.button(6).whileTrue(new InstantCommand(() -> {pulley.enable(); lift.enable(); gripperAngle.enable();}));



        //Manual control commands
        joystickHandler4.button(2).whileTrue(new InstantCommand(() -> {lift.setSpeed(0.2); 
                                                                                    lift.setMode(false);}))
                                                .onFalse(new InstantCommand(() -> {lift.setSpeed(0); 
                                                                                    lift.setMode(true);}));

        joystickHandler4.button(3).whileTrue(new InstantCommand(() -> {lift.setSpeed(-0.2); 
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

        joystickHandler4.povLeft().whileTrue(new InstantCommand(() -> {gripperAngle.setSpeed(0.5); 
                                                                        gripperAngle.setMode(false);}))
                                    .onFalse(new InstantCommand(() -> {gripperAngle.setSpeed(0); 
                                                                        gripperAngle.setMode(true);}));

        joystickHandler4.povRight().whileTrue(new InstantCommand(() -> {gripperAngle.setSpeed(-0.5); 
                                                                        gripperAngle.setMode(false);}))
                                    .onFalse(new InstantCommand(() -> {gripperAngle.setSpeed(0); 
                                                                        gripperAngle.setMode(true);}));


        //Records current position of lift/arm system
        //joystickHandler4.button(10).whileTrue(new InstantCommand(() -> {positionHandler.capturePose();}));

        //Cycling through presets
        joystickHandler4.povUp().whileTrue(new InstantCommand(() -> {positionHandler.increaseIndex();})).whileFalse(new InstantCommand(() -> positionHandler.setPose()));

        joystickHandler4.povDown().whileTrue(new InstantCommand(() -> {positionHandler.decreaseIndex();})).whileFalse(new InstantCommand(() -> positionHandler.setPose()));
    }
}