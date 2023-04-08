package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoCommandGroup;
import frc.robot.commands.FieldSpaceDrive;
import frc.robot.commands.LineUp;
import frc.robot.commands.RobotSpaceDrive;
import frc.robot.commands.StrafeLeft;
import frc.robot.commands.StrafeRight;
import frc.robot.subsystems.GripperAngleSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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
    //public LEDSubsystem LEDs;

    private FieldSpaceDrive fieldSpaceDriveCommand;
    private RobotSpaceDrive robotSpaceDriveCommand;
    private PositionHandler positionHandler;
    private PathHandler pathHandler;
    private LineUp left;
    private LineUp right;
    private LineUp pickup;

    private AutoCommandGroup auto;

    private JoystickHandler joy3;
    private JoystickHandler joy4;

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


        joy3 = new JoystickHandler(3);
        joy4 = new JoystickHandler(4);

        // Field space uses pigeon2 to get its angle
        fieldSpaceDriveCommand = new FieldSpaceDrive(swerveDrive, joy3, pigeon);
        robotSpaceDriveCommand = new RobotSpaceDrive(swerveDrive, joy3);
        swerveDrive.setDefaultCommand(fieldSpaceDriveCommand);

        left = new LineUp(swerveDrive, limeLight, pigeon, "left");
        right = new LineUp(swerveDrive, limeLight, pigeon, "right");
        pickup = new LineUp(swerveDrive, limeLight, pigeon, "pickup");

        positionHandler = new PositionHandler(lift, pulley, gripperAngle);
        pathHandler = new PathHandler(swerveDrive);

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
        auto = new AutoCommandGroup(this, autoChooser.getSelected());
        return auto;
    }

    private void configureButtonBindings() {

        //Gripper toggle
        joy3.button(1).whileTrue(new InstantCommand(() -> pneumatics.togglePneumaticState()));
        
        //Auto balance
        //joystickHandler3.button(2).whileTrue(new SequentialCommandGroup(unbalance, rebalance));

        //This will set the current orientation to be "forward" for field drive
        joy3.button(3).whileTrue(new InstantCommand(() -> fieldSpaceDriveCommand.zero()));

        //This will set the current orientation to be "backward" for field drive
        joy3.button(4).whileTrue(new InstantCommand(() -> pigeon.setYaw(180)));

        // This will toggle slow mode for drive
        joy3.button(11).whileTrue(new InstantCommand(() -> fieldSpaceDriveCommand.toggleSlowMode())).onFalse(new InstantCommand(() -> fieldSpaceDriveCommand.toggleSlowMode()));

        //Snow plow break
        joy3.button(6).whileTrue(new InstantCommand(() -> {swerveDrive.brake(); fieldSpaceDriveCommand.drive(false);})).onFalse(new InstantCommand(() -> fieldSpaceDriveCommand.drive(true)));

        //Move to score high on left node 
        joy3.button(10).whileTrue(left).whileTrue(new InstantCommand(() -> positionHandler.setPose(4)));
        
        //Move to score low on left node 
        joy3.button(9).whileTrue(left).whileTrue(new InstantCommand(() -> positionHandler.setPose(3)));

        //Move to score high on right node 
        joy3.button(14).whileTrue(right).whileTrue(new InstantCommand(() -> positionHandler.setPose(4)));
        
        //Move to score low on right node 
        joy3.button(13).whileTrue(right).whileTrue(new InstantCommand(() -> positionHandler.setPose(3)));

        //Pickup
        joy3.button(12).whileTrue(pickup).whileTrue(new InstantCommand(() -> positionHandler.setPose(1)));






        
        
        //Soft disable for lift/arm
        joy4.button(5).whileTrue(new InstantCommand(() -> {pulley.disable(); lift.disable(); gripperAngle.disable();})).onFalse(new InstantCommand(() -> {pulley.enable(); lift.enable(); gripperAngle.enable();}));



        //Manual control commands
        joy4.button(2).whileTrue(new InstantCommand(() -> {lift.setSpeed(0.2); 
                                                                                    lift.setMode(false);}))
                                                .onFalse(new InstantCommand(() -> {lift.setSpeed(0); 
                                                                                    lift.setMode(true);}));

        joy4.button(3).whileTrue(new InstantCommand(() -> {lift.setSpeed(-0.2); 
                                                                                    lift.setMode(false);}))
                                                .onFalse(new InstantCommand(() -> {lift.setSpeed(0); 
                                                                                    lift.setMode(true);}));


        joy4.button(4).whileTrue(new InstantCommand(() -> { pulley.increment();}));

        joy4.button(1).whileTrue(new InstantCommand(() -> { pulley.decrement();}));


        joy4.povLeft().whileTrue(new InstantCommand(() -> {gripperAngle.setSpeed(0.5); 
                                                                        gripperAngle.setMode(false);}))
                                    .onFalse(new InstantCommand(() -> {gripperAngle.setSpeed(0); 
                                                                        gripperAngle.setMode(true);}));

        joy4.povRight().whileTrue(new InstantCommand(() -> {gripperAngle.setSpeed(-0.5); 
                                                                        gripperAngle.setMode(false);}))
                                    .onFalse(new InstantCommand(() -> {gripperAngle.setSpeed(0); 
                                                                        gripperAngle.setMode(true);}));

        joy4.button(6).whileTrue(new InstantCommand(() -> positionHandler.setPose(2)));
        
        joy4.button(10).whileTrue(new InstantCommand(() -> positionHandler.setPose(5)));

        //Cycling through presets
        joy4.povUp().whileTrue(new InstantCommand(() -> {positionHandler.increaseIndex();})).whileFalse(new InstantCommand(() -> positionHandler.setPose()));

        joy4.povDown().whileTrue(new InstantCommand(() -> {positionHandler.decreaseIndex();})).whileFalse(new InstantCommand(() -> positionHandler.setPose()));
    }
}