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
import frc.robot.commands.AutoCommandGroup;
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

    private AutoCommandGroup auton;

    private JoystickHandler joystickHandler3;
    private JoystickHandler joystickHandler4;

    private SendableChooser<Integer> autoChooser;


    public SwerveDriveSubsystem getSwervedriveSubsystem() {
        return swerveDrive;
    }

    public Pigeon2Handler getPigeon2Handler() {
        return pigeon;
    }

    public PneumaticSubsystem getPneumatics(){
        return pneumatics;
    }

    public PulleySubsystem getPulley(){
        return pulley;
    }

    public LiftSubsystem getLift(){
        return lift;
    }

    public GripperAngleSubsystem getGripperAngle(){
        return gripperAngle;
    }

    public PositionHandler getPositionHandler(){
        return positionHandler;
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

        autoChooser = new SendableChooser<Integer>();
        autoChooser.setDefaultOption("Just Score", 1);
        autoChooser.addOption("Backup", 2);
        autoChooser.addOption("Balance", 3);
        autoChooser.addOption("Backup and Floor Pose", 4);
        autoChooser.addOption("Backup and Start Pose", 5);
        autoChooser.addOption("Start Pose", 6);
        
    
        Shuffleboard.getTab("Preferences").add("Autonomous", autoChooser);

        pigeon.setYaw(180);
        pigeon.setPitchOffset(pigeon.getPitch());
        pigeon.setRollOffset(pigeon.getRoll());
        //fieldSpaceDriveCommand.zero();
        configureButtonBindings();
    }

    public Command getAutoCommandGroup() {
        auton = new AutoCommandGroup(this, autoChooser.getSelected());
        return auton;
    }

    private void configureButtonBindings() {

        //Gripper toggle
        joystickHandler3.button(1).whileTrue(new InstantCommand(() -> pneumatics.togglePneumaticState()));
        
        //Auto balance
        joystickHandler3.button(2).whileTrue(new SequentialCommandGroup(unbalance, rebalance));

        //This will set the current orientation to be "forward" for field drive
        joystickHandler3.button(3).whileTrue(new InstantCommand(() -> fieldSpaceDriveCommand.zero()));

        // This will reset odometry for Swerve drive
        //joystickHandler3.button(4).whileTrue(new InstantCommand(() -> swerveDriveSubsystem.resetPose()));

        //Snow plow break
        joystickHandler3.button(6).whileTrue(new InstantCommand(() -> {swerveDrive.brake(); fieldSpaceDriveCommand.drive(false);})).onFalse(new InstantCommand(() -> fieldSpaceDriveCommand.drive(true)));

        //Move to score +1 node
        joystickHandler3.povLeft().whileTrue(new LineUp(swerveDrive, limeLight, "left"));           

        //Move to score -1 node
        joystickHandler3.povRight().whileTrue(new LineUp(swerveDrive, limeLight, "right"));

        //Move to score center node
        joystickHandler3.povUp().whileTrue(new LineUp(swerveDrive, limeLight, "center"));

        //Pickup
        joystickHandler3.button(10).whileTrue(new LineUp(swerveDrive, limeLight, "pickup"));






        
        
        //Soft disable for lift/arm
        joystickHandler4.button(5).whileTrue(new InstantCommand(() -> {pulley.disable(); lift.disable(); gripperAngle.disable();})).onFalse(new InstantCommand(() -> {pulley.enable(); lift.enable(); gripperAngle.enable();}));



        //Manual control commands
        joystickHandler4.button(2).whileTrue(new InstantCommand(() -> {lift.setSpeed(0.2); 
                                                                                    lift.setMode(false);}))
                                                .onFalse(new InstantCommand(() -> {lift.setSpeed(0); 
                                                                                    lift.setMode(true);}));

        joystickHandler4.button(3).whileTrue(new InstantCommand(() -> {lift.setSpeed(-0.2); 
                                                                                    lift.setMode(false);}))
                                                .onFalse(new InstantCommand(() -> {lift.setSpeed(0); 
                                                                                    lift.setMode(true);}));


        joystickHandler4.button(4).whileTrue(new InstantCommand(() -> { pulley.increment();}));

        joystickHandler4.button(1).whileTrue(new InstantCommand(() -> { pulley.decrement();}));


        joystickHandler4.povLeft().whileTrue(new InstantCommand(() -> {gripperAngle.setSpeed(0.5); 
                                                                        gripperAngle.setMode(false);}))
                                    .onFalse(new InstantCommand(() -> {gripperAngle.setSpeed(0); 
                                                                        gripperAngle.setMode(true);}));

        joystickHandler4.povRight().whileTrue(new InstantCommand(() -> {gripperAngle.setSpeed(-0.5); 
                                                                        gripperAngle.setMode(false);}))
                                    .onFalse(new InstantCommand(() -> {gripperAngle.setSpeed(0); 
                                                                        gripperAngle.setMode(true);}));


        //Cycling through presets
        joystickHandler4.povUp().whileTrue(new InstantCommand(() -> {positionHandler.increaseIndex();})).whileFalse(new InstantCommand(() -> positionHandler.setPose()));

        joystickHandler4.povDown().whileTrue(new InstantCommand(() -> {positionHandler.decreaseIndex();})).whileFalse(new InstantCommand(() -> positionHandler.setPose()));
    }
}