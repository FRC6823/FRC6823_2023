package frc.robot;

import java.util.*;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.FieldSpaceDrive;
import frc.robot.commands.RobotSpaceDrive;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.Constants;

public class RobotContainer {
    // test commit
    public SwerveDriveSubsystem swerveDriveSubsystem;
    public Pigeon2Handler pigeon;

    private FieldSpaceDrive fieldSpaceDriveCommand;
    private RobotSpaceDrive robotSpaceDriveCommand;

    private JoystickHandler joystickHandler3;
    //private JoystickHandler joystickHandler4;

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
        //joystickHandler4 = new JoystickHandler(4);

        // Field space uses pigeon2 to get its angle
        fieldSpaceDriveCommand = new FieldSpaceDrive(swerveDriveSubsystem, joystickHandler3, pigeon);
        robotSpaceDriveCommand = new RobotSpaceDrive(swerveDriveSubsystem, joystickHandler3);
        swerveDriveSubsystem.setDefaultCommand(fieldSpaceDriveCommand);
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
        //pigeon.zeroYaw();
        
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
        // Hold button 8 to set the swerve just forward, this is for calibration
        // purposes
        
        //      joystickHandler3.button(8).whileTrue(swerveDriveSubsystem.drive(0, 0.1, 0));

        // This will set the current orientation to be "forward" for field drive
        joystickHandler3.button(3).whileTrue(new InstantCommand(() -> fieldSpaceDriveCommand.zero()));
        // This will reset odometry for Swerve drive
        joystickHandler3.button(4).whileTrue(new InstantCommand(() -> swerveDriveSubsystem.resetPose()));
        // Holding 7 will enable robot space drive, instead of field space
        joystickHandler3.button(2).whileTrue(robotSpaceDriveCommand);

        joystickHandler3.button(6).whileTrue(new InstantCommand(() -> swerveDriveSubsystem.resetSensors()));

        joystickHandler3.button(1).whileTrue(new InstantCommand(() -> {swerveDriveSubsystem.brake(); fieldSpaceDriveCommand.drive(false);})).onFalse(new InstantCommand(() -> fieldSpaceDriveCommand.drive(true)));
    }

    public Command getAutonomousCommand(){
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.kMaxSpeed, Constants.kMaxAccel);
        //trajectoryConfig.setReversed(true);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory
            (new Pose2d(0, 0, new Rotation2d(0.0)), 
            List.of(new Translation2d(3,0)), 
             new Pose2d(1, 0, new Rotation2d(0.0)), trajectoryConfig);
        
        PIDController xController = new PIDController(.5, 0.00001, 0);
        PIDController yController = new PIDController(.5, 0.00001, 0);
        ProfiledPIDController turnController = new ProfiledPIDController(0.1, 0, 0, Constants.kTurnControlConstraints);
        turnController.enableContinuousInput(Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveDriveSubsystem::getRobotPose, swerveDriveSubsystem.getKinematics(), xController, yController, turnController, swerveDriveSubsystem::setSwerveModuleStates, swerveDriveSubsystem);

        return new SequentialCommandGroup (
            new InstantCommand(() -> swerveDriveSubsystem.resetPose()),
            swerveControllerCommand,
            new InstantCommand(() -> {swerveDriveSubsystem.brake();})
        );
    }
}