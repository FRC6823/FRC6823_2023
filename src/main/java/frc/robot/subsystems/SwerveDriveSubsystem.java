package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableRegistry;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
//import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pigeon2Handler;
import frc.robot.util.Constants;

// import java.util.Map;

public class SwerveDriveSubsystem extends SubsystemBase {

    public final double L = 0.5334 / 2;
    public final double W = 0.6858 / 2;
    //public final double L = 21;
    //public final double W = 27; // These are from the Length and Width between wheels.
    // CHANGE THESE IF THE ROBOT IS NOT A SQUARE

    private SwerveWheelModuleSubsystem backRight;
    private SwerveWheelModuleSubsystem backLeft;
    private SwerveWheelModuleSubsystem frontRight;
    private SwerveWheelModuleSubsystem frontLeft;
    private SwerveDriveKinematics kinematics;
    private ChassisSpeeds speeds;
    private PIDController angleController;
    private Pigeon2Handler pigeon;
    private SwerveDriveOdometry odometry;
    
    public SwerveDriveSubsystem(Pigeon2Handler pigeon) {
        //calibrateWidget = Shuffleboard.getTab("Preferences").addPersistent("Calibrate?", false)
                //.withWidget(BuiltInWidgets.kToggleButton);
        // invertWidget = Shuffleboard.getTab("Preferences").addPersistent("Invert?", false)
        //         .withWidget(BuiltInWidgets.kToggleButton);

        backRight = new SwerveWheelModuleSubsystem(1, 8, 26, "BR", Constants.bROffset);// These are the motors and encoder
                                                                // CAN IDs for swerve drive
        backLeft = new SwerveWheelModuleSubsystem(3, 2, 27, "BL", Constants.bLOffset);
        frontRight = new SwerveWheelModuleSubsystem(5, 4, 28, "FR", Constants.fROffset);
        frontLeft = new SwerveWheelModuleSubsystem(7, 6, 25, "FL", Constants.fLOffset);// The order is angle, speed,
                                                                                   // encoder
        SendableRegistry.addChild(this, backRight);
        SendableRegistry.addChild(this, backLeft);
        SendableRegistry.addChild(this, frontRight);
        SendableRegistry.addChild(this, frontLeft);

        SendableRegistry.addLW(this, "Swerve Drive Subsystem");

        angleController = new PIDController(.3, 0, 0);
        angleController.enableContinuousInput(0, Math.PI * 2);
        angleController.setSetpoint(0);
        
        Translation2d backRightLocation = new Translation2d(-W, -L);
        Translation2d backLeftLocation = new Translation2d(-W, L);
        Translation2d frontRightLocation = new Translation2d(W, -L);
        Translation2d frontLeftLocation = new Translation2d(W, L);

        kinematics = new SwerveDriveKinematics(backRightLocation, backLeftLocation, frontRightLocation, frontLeftLocation);
        speeds = new ChassisSpeeds(0, 0, 0);
        this.pigeon = pigeon;
        odometry = new SwerveDriveOdometry
                    (kinematics, 

                    pigeon.getAngleRad(), 

                    new SwerveModulePosition[] {
                        backRight.getSwerveModulePosition(), 
                        backLeft.getSwerveModulePosition(),
                        frontRight.getSwerveModulePosition(),
                        frontLeft.getSwerveModulePosition()});
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        // implementation from Pride of the North
        speeds = chassisSpeeds;
    }

    // @Override
    public void periodic() {
        // Convert to module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 5.5);
        // Front left module state
        SwerveModuleState backRightState = moduleStates[0];

        // Front right module state
        SwerveModuleState backLeftState = moduleStates[1];

        // Back left module state
        SwerveModuleState frontRightState = moduleStates[2];

        // Back right module state
        SwerveModuleState frontLeftState = moduleStates[3];

        backLeft.drive(backLeftState.speedMetersPerSecond, backLeftState.angle.getDegrees()); //5.5 m/s is maximum zero load velocity
        backRight.drive(-backRightState.speedMetersPerSecond, backRightState.angle.getDegrees());
        frontLeft.drive(frontLeftState.speedMetersPerSecond, frontLeftState.angle.getDegrees());
        frontRight.drive(-frontRightState.speedMetersPerSecond, frontRightState.angle.getDegrees());

        odometry.update(pigeon.getAngleRad(), 
                        new SwerveModulePosition[] {
                            backRight.getSwerveModulePosition(), 
                            backLeft.getSwerveModulePosition(),
                            frontRight.getSwerveModulePosition(),
                            frontLeft.getSwerveModulePosition()});

        SmartDashboard.putNumber("PoseX", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Pose Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Pose Theta", odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("Pigeon Readings", pigeon.getAngleDeg().getDegrees());
    }

    public void stop() {
        backRight.stop();
        backLeft.stop();
        frontRight.stop();
        frontLeft.stop();
    }

    public void coast(){
        backRight.coast();
        backLeft.coast();
        frontRight.coast();
        frontLeft.coast();
    }

    public void brake(){
        backRight.brake();
        backLeft.brake();
        frontRight.brake();
        frontLeft.brake();
    }

    //public Rotation2d getRobotAngle()
    //{
        //return pigeon.getAngleDeg();
    //}

    public void resetPose()
    {
        odometry.resetPosition(pigeon.getAngleRad(), 
                                new SwerveModulePosition[] {
                                    backRight.getSwerveModulePosition(), 
                                    backLeft.getSwerveModulePosition(),
                                    frontRight.getSwerveModulePosition(),
                                    frontLeft.getSwerveModulePosition()},
                                new Pose2d(0, 0, pigeon.getAngleRad()));
    }

    public void resetSensors()
    {
        backLeft.resetSensor();
        backRight.resetSensor();
        frontLeft.resetSensor();
        frontRight.resetSensor();
    }
}

