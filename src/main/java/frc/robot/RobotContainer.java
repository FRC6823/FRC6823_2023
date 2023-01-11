package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.commands.Shoot;
import frc.robot.commands.SwitchPipelineCommand;
import frc.robot.commands.TargetSpaceDrive;
import frc.robot.commands.Load;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class RobotContainer {
    // test commit
    public SwerveDriveSubsystem swerveDriveSubsystem;
    public NavXHandler navX;
    public ShooterSubsystem shooterSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public ConveyorSubsystem conveyorSubsystem;
    public Shoot shoot;
    public Load backLoad;
    public LiftSubsystem liftSubsystem;

    private FieldSpaceDrive fieldSpaceDriveCommand;
    private RobotSpaceDrive robotSpaceDriveCommand;
    private TargetSpaceDrive targetSpaceDriveCommand;
    private AutoCommandGroup auton;

    private JoystickHandler joystickHandler3;
    private JoystickHandler joystickHandler4;
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

    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public ConveyorSubsystem getConveyorSubsystem() {
        return conveyorSubsystem;
    }

    public LiftSubsystem getLiftSubsystem() {
        return liftSubsystem;
    }

    public RobotContainer() {
        swerveDriveSubsystem = new SwerveDriveSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        joystickHandler3 = new JoystickHandler(3);
        joystickHandler4 = new JoystickHandler(4);
        limeLightSubsystem = new LimeLightSubsystem(8);
        intakeSubsystem = new IntakeSubsystem();
        conveyorSubsystem = new ConveyorSubsystem();
        liftSubsystem = new LiftSubsystem();

        navX = new NavXHandler(); // navx input

        // Field space uses navx to get its angle
        fieldSpaceDriveCommand = new FieldSpaceDrive(swerveDriveSubsystem, joystickHandler3, navX);
        robotSpaceDriveCommand = new RobotSpaceDrive(swerveDriveSubsystem, joystickHandler3);
        targetSpaceDriveCommand = new TargetSpaceDrive(swerveDriveSubsystem, joystickHandler3, limeLightSubsystem, navX);
        backLoad = new Load(shooterSubsystem, conveyorSubsystem);
        swerveDriveSubsystem.setDefaultCommand(fieldSpaceDriveCommand);
        //swerveDriveSubsystem.setDefaultCommand(targetSpaceDriveCommand);

        shoot = new Shoot(shooterSubsystem, conveyorSubsystem, joystickHandler4);
        shooterSubsystem.setDefaultCommand(shoot); // Check shoot for shoot button mapping

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

        joystickHandler4.button(6).whileTrue(backLoad);

        //joystickHandler4.button(6).whenReleased(backLoad::stop); 
        //do we still need this? i'm pretty sure while true takes care of this step. 

        //joystickHandler4.button(1).whileActiveContinuous(() -> intakeSubsystem.backAngle(), intakeSubsystem)
                //.whenInactive(intakeSubsystem::stopAngle);

                joystickHandler4.button(1).whileTrue(() -> intakeSubsystem.backAngle());
                
        //joystickHandler4.button(2).whileActiveContinuous(() -> intakeSubsystem.intake(), intakeSubsystem)
                //.whenInactive(intakeSubsystem::stopIntake);    

                joystickHandler4.button(2).whileTrue(() -> intakeSubsystem.intake());

        //joystickHandler4.button(3).whileActiveContinuous(() -> intakeSubsystem.backIntake(), intakeSubsystem)
                //.whenInactive(intakeSubsystem::stopIntake);

                joystickHandler4.button(3).whileTrue(() -> intakeSubsystem.backIntake());

        //joystickHandler4.button(4).whileActiveContinuous(() -> intakeSubsystem.angle(), intakeSubsystem)
                //.whenInactive(intakeSubsystem::stopAngle);

                joystickHandler4.button(4).whileTrue(() -> intakeSubsystem.angle());

        //joystickHandler4.button(2).whileActiveContinuous(() -> conveyorSubsystem.convey(), conveyorSubsystem)
                //.whenInactive(conveyorSubsystem::stopConvey);

                joystickHandler4.button(2).whileTrue(() -> conveyorSubsystem.convey());

        //joystickHandler4.button(3).whileActiveContinuous(() -> conveyorSubsystem.backConvey(), conveyorSubsystem)
                //.whenInactive(conveyorSubsystem::stopConvey);

                joystickHandler4.button(3).whileTrue(() -> conveyorSubsystem.backConvey());

        //joystickHandler4.button(7).whenPressed(() -> swerveDriveSubsystem.autoCali(), swerveDriveSubsystem);
        // joystickHandler4.button(8).whileHeld(() ->
        // shooterSubsystem.setShooterAngle(30), shooterSubsystem);
        joystickHandler3.button(1).whileTrue(
        liftSubsystem.liftUp());

        joystickHandler3.button(6).whileTrue(() ->
        liftSubsystem.liftDown());

        joystickHandler3.button(9).whileTrue(() ->
        liftSubsystem.leftUp());

        joystickHandler3.button(10).whileTrue(() ->
        liftSubsystem.leftDown());

        joystickHandler3.button(11).whileTrue(() ->
        liftSubsystem.rightUp());

        joystickHandler3.button(12).whileTrue(() ->
        liftSubsystem.rightDown());

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
