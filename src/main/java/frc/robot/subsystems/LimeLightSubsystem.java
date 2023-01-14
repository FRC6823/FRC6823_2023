package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashSet;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightTools;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightSubsystem extends SubsystemBase {

    //Declare Servo limelight is attatched to, Network Table, 
    //last known object position on limelight view, and directions 
    //of different landmarks
    private Servo servo;
    private NetworkTable table;
    private double lastKnownZ;
    private double lastKnownX;

    public LimeLightSubsystem(int servo) {
        //Instantiate Network Table to limelight Network Table and servo
        table = NetworkTableInstance.getDefault().getTable("limelight");
        this.servo = new Servo(servo);
        SendableRegistry.addLW(this, "LimeLight Subsystem");
    }

    public HashSet<Subsystem> setPipeline(int pipeline) {
        HashSet<Subsystem> tree = new HashSet<Subsystem>();
        tree.add(this);
        table.getEntry("pipeline").setNumber(pipeline);
        if (pipeline == 0)
            setServoAngle(70);
        else
            setServoAngle(0); //ground //was 30

        return tree;
    }

    public double getServoAngle() {
        SmartDashboard.putNumber("LL servo angle", this.servo.getAngle());
        return this.servo.getAngle();
    }

    public double getServoAngleFromGroundRad() { //returns angle from ground in radians
        return ((this.servo.getAngle() - 40) / 360.0) * (2 * Math.PI);
    }

    public int getPipeline() {
        return (int) table.getEntry("getpipe").getDouble(0);
    }

    public void setServoAngle(double degrees) {
        servo.setAngle(degrees);
    }

    // "T"x is for theta, aka this is from 2d pipelines, non "T" methods get 3d
    // pipline functions.
    public double getTx() {
        return table.getEntry("tx").getDouble(0);
    }

    public double getTxRad() {
        return (table.getEntry("tx").getDouble(0) / 360.0) * (2.0 * Math.PI);
    }

    public double getTy() {
        return table.getEntry("ty").getDouble(0);
    }

    public double getTyRad() {
        return (table.getEntry("ty").getDouble(0) / 360.0) * (2.0 * Math.PI);
    }

    // this is the 3d Distance, should always be negative
    public double getZ() {
        double newZ = table.getEntry("camtran").getDoubleArray(new double[] { 0 })[2];
        if (newZ < -15 && newZ > -300) {
            lastKnownZ = newZ;
            lastKnownX = table.getEntry("camtran").getDoubleArray(new double[] { 0 })[0];
        }
        return lastKnownZ;
    }

    // this is the 3d strafe
    public double getX() {
        return lastKnownX;
    }

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) != 0;
    }

    @Override
    public void periodic() {
        // setServoAngle(0);
        SmartDashboard.putNumber("Servo", servo.getPosition());
        SmartDashboard.putNumber("Servo_Angle", getServoAngle());
        SmartDashboard.putNumber("tx", getTx());
        SmartDashboard.putNumber("ty", getTy());
        SmartDashboard.putNumber("pipeline", table.getEntry("pipeline").getDouble(0));
        SmartDashboard.putNumber("Dist from tower", LimelightTools.distFromTower(getTyRad() / Math.PI));
    }
}