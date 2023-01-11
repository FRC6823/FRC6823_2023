package frc.robot.subsystems;

import java.util.TreeSet;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
//import edu.wpi.first.wpilibj.Preferences;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

// import java.util.Map;

public class SwerveDriveSubsystem extends SubsystemBase {
    /**
     * This subsystem does calculations to take controller inputs and
     * convert them into rotation and speed values for each motor (which is
     * controlled via the SwerveWheelModuleSubsystem class)
     * <p>
     * This code heavily attributed from Jacob Misirian of FIRST Robotics Team 2506
     * of Franklin, WI.
     */
    public final double L = 1;
    public final double W = 1; // These are from the Length and Width between wheels.
    // CHANGE THESE IF THE ROBOT IS NOT A SQUARE

    private SwerveWheelModuleSubsystem backRight;
    private SwerveWheelModuleSubsystem backLeft;
    private SwerveWheelModuleSubsystem frontRight;
    private SwerveWheelModuleSubsystem frontLeft;
    private SwerveDriveKinematics kinematics;

    private PIDController angleController;
    private double fieldangle = 0; //
    // private SimpleWidget FLAngle;
    // private SimpleWidget FRAngle;
    // private SimpleWidget BLAngle;
    // private SimpleWidget BRAngle;
    private SimpleWidget calibrateWidget;
    //private SimpleWidget invertWidget;

    public void setFieldAngle(double fieldangle) {
        this.fieldangle = fieldangle;
        angleController.setSetpoint(this.fieldangle);

    }

    public SwerveDriveSubsystem() {
        calibrateWidget = Shuffleboard.getTab("Preferences").addPersistent("Calibrate?", false)
                .withWidget(BuiltInWidgets.kToggleButton);
        // invertWidget = Shuffleboard.getTab("Preferences").addPersistent("Invert?", false)
        //         .withWidget(BuiltInWidgets.kToggleButton);

        backRight = new SwerveWheelModuleSubsystem(1, 8, 26, "BR", calibrateWidget, Constants.bROffset);// These are the motors and encoder
                                                                // CAN IDs for swerve drive
        backLeft = new SwerveWheelModuleSubsystem(3, 2, 27, "BL", calibrateWidget, Constants.bLOffset);
        frontRight = new SwerveWheelModuleSubsystem(5, 4, 28, "FR", calibrateWidget, Constants.fROffset);
        frontLeft = new SwerveWheelModuleSubsystem(7, 6, 25, "FL", calibrateWidget, Constants.fLOffset);// The order is angle, speed,
                                                                                   // encoder, calibrateWidget
        SendableRegistry.addChild(this, backRight);
        SendableRegistry.addChild(this, backLeft);
        SendableRegistry.addChild(this, frontRight);
        SendableRegistry.addChild(this, frontLeft);

        SendableRegistry.addLW(this, "Swerve Drive Subsystem");

        angleController = new PIDController(.3, 0, 0);
        angleController.enableContinuousInput(0, Math.PI * 2);
        angleController.setSetpoint(0);
        //SmartDashboard.putString("Ready Call", "Autobots, Roll Out!");
        // FLAngle = Shuffleboard.getTab("Calibrate").addPersistent("FLAngle", 0).withWidget(BuiltInWidgets.kNumberSlider)
        //         .withProperties(Map.of("min", 0, "max", 360));
        // FRAngle = Shuffleboard.getTab("Calibrate").addPersistent("FRAngle", 0).withWidget(BuiltInWidgets.kNumberSlider)
        //         .withProperties(Map.of("min", 0, "max", 360));
        // BLAngle = Shuffleboard.getTab("Calibrate").addPersistent("BLAngle", 0).withWidget(BuiltInWidgets.kNumberSlider)
        //         .withProperties(Map.of("min", 0, "max", 360));
        // BRAngle = Shuffleboard.getTab("Calibrate").addPersistent("BRAngle", 0).withWidget(BuiltInWidgets.kNumberSlider)
        //         .withProperties(Map.of("min", 0, "max", 360));
        //autoCaliZero();

        // Locations for the swerve drive modules relative to the robot center.
        Translation2d backRightLocation = new Translation2d(1, -1);
        Translation2d backLeftLocation = new Translation2d(-1, -1);
        Translation2d frontRightLocation = new Translation2d(1, 1);
        Translation2d frontLeftLocation = new Translation2d(-1, 1);

        kinematics = new SwerveDriveKinematics(backRightLocation, backLeftLocation, frontRightLocation, frontLeftLocation);
    }

    public TreeSet<Subsystem> drive(double x1, double y1, double x2) {
        
        TreeSet<Subsystem> tree = new TreeSet<Subsystem>();
        tree.add(this);

        ChassisSpeeds speeds = new ChassisSpeeds(x1 * 715, y1 * 715, -x2 * 715); //715.23 radians/second is the top no load speed of the motors

        // Convert to module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        // Front left module state
        SwerveModuleState backRightState = moduleStates[0];

        // Front right module state
        SwerveModuleState backLeftState = moduleStates[1];

        // Back left module state
        SwerveModuleState frontRightState = moduleStates[2];

        // Back right module state
        SwerveModuleState frontLeftState = moduleStates[3];

        backLeft.drive(backLeftState.speedMetersPerSecond / 5.5, MathUtil.angleModulus(backLeftState.angle.getDegrees())/Math.PI); //5.5 m/s is maximum zero load velocity
        backRight.drive(backRightState.speedMetersPerSecond / 5.5, MathUtil.angleModulus(backRightState.angle.getDegrees())/Math.PI);
        frontLeft.drive(frontLeftState.speedMetersPerSecond / 5.5, MathUtil.angleModulus(frontLeftState.angle.getDegrees())/Math.PI);
        frontRight.drive(frontRightState.speedMetersPerSecond / 5.5, MathUtil.angleModulus(frontRightState.angle.getDegrees())/Math.PI);

        // double r = Math.sqrt((L * L) + (W * W)); // diagonal of robot
        // double backRightSpeed;
        // double backLeftSpeed;
        // double frontRightSpeed;
        // double frontLeftSpeed;

        // double backRightAngle;
        // double backLeftAngle;
        // double frontRightAngle;
        // double frontLeftAngle;

        // // From here to the next comment sets each module to the <x,y> from
        // // the joystick plus rotation times the diagonal to that module
        // // (i.e. FR is set to <x1,y1> + x2*<diagonal BL->FR>)
        // double a = x1 - x2 * (L / r);
        // double b = x1 + x2 * (L / r);
        // double c = y1 - x2 * (W / r);
        // double d = y1 + x2 * (W / r);

        // backRightSpeed = Math.sqrt((b * b) + (c * c));
        // backLeftSpeed = Math.sqrt((a * a) + (c * c));
        // frontRightSpeed = Math.sqrt((b * b) + (d * d));
        // frontLeftSpeed = Math.sqrt((a * a) + (d * d));

        // frontRightAngle = Math.atan2(b, c) / Math.PI;
        // backRightAngle = Math.atan2(a, c) / Math.PI;
        // frontLeftAngle = Math.atan2(b, d) / Math.PI;
        // backLeftAngle = Math.atan2(a, d) / Math.PI;

        // if (invertWidget.getEntry().getBoolean(false)){
        //     backLeft.drive(-backLeftSpeed, -backLeftAngle);
        //     backRight.drive(backRightSpeed, -backRightAngle);
        //     frontRight.drive(frontRightSpeed, -frontRightAngle);
        //     frontLeft.drive(-frontLeftSpeed, -frontLeftAngle);
        // }else{
        //     backLeft.drive(backLeftSpeed, -backLeftAngle);
        //     backRight.drive(-backRightSpeed, -backRightAngle);
        //     frontRight.drive(-frontRightSpeed, -frontRightAngle);
        //     frontLeft.drive(frontLeftSpeed, -frontLeftAngle);
        // }

        // //Print speed values
        // SmartDashboard.putNumber("Backright Speed", backRightSpeed);
        // SmartDashboard.putNumber("Backleft Speed", backLeftSpeed);
        // SmartDashboard.putNumber("Frontright Speed", frontRightSpeed);
        // SmartDashboard.putNumber("Frontleft Speed", frontLeftSpeed);

        return tree;
    }

    // @Override
    public void periodic() {
        // Do NOT make negative!!!!
        // adding is counter clockwise, subtratcting is clockwise?
        // frontLeft.setZero(FLAngle.getEntry().getDouble(0));
        // frontRight.setZero(FRAngle.getEntry().getDouble(0));
        // backLeft.setZero(BLAngle.getEntry().getDouble(0));
        // backRight.setZero(BRAngle.getEntry().getDouble(0));
    }

    public void stop() {
        backRight.stop();
        backLeft.stop();
        frontRight.stop();
        frontLeft.stop();
    }

    // public void autoCali() {
    //     FLAngle.getEntry().setDouble(frontLeft.autoCali());
    //     FRAngle.getEntry().setDouble(frontRight.autoCali());
    //     BLAngle.getEntry().setDouble(backLeft.autoCali());
    //     BRAngle.getEntry().setDouble(backRight.autoCali());
    // }

    // public void autoCaliZero(){
    //     if (frontLeft.autoCali() != -2){
    //         FLAngle.getEntry().setNumber(frontLeft.autoCaliZero());
    //         FRAngle.getEntry().setNumber(frontRight.autoCaliZero());
    //         BLAngle.getEntry().setNumber(backLeft.autoCaliZero());
    //         BRAngle.getEntry().setNumber(backRight.autoCaliZero());
    //     }
    // }

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
}
