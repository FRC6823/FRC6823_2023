package frc.robot;

//import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class Pigeon2Handler {
    private WPI_Pigeon2 pigeon;
    //private double initialAngle;

    //public double getInitialAngle() {
        //return initialAngle;
    //}

    //public void setInitialAngle() {
        //initialAngle = getAngleRad();
    //}

    public WPI_Pigeon2 getAhrs() {
        return pigeon;
    }

    public Pigeon2Handler() {
        try {
            /***********************************************************************
             * pigeon2-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
             * http://pigeon2-mxp.kauailabs.com/guidance/selecting-an-interface.
             ***********************************************************************/
            pigeon = new WPI_Pigeon2(18);
            pigeon.configMountPose(-90,0, 0 ,100);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating pigeon:  " + ex.getMessage(), true);
        }
    }

    public void printEverything() {
        // SmartDashboard.putNumber("getAngle()", ahrs.getAngle());

        // SmartDashboard.putNumber("getDisplacementX()", ahrs.getDisplacementX());
        // SmartDashboard.putNumber("getDisplacementY()", ahrs.getDisplacementY());
        // SmartDashboard.putNumber("getDisplacementZ()", ahrs.getDisplacementZ());

        // SmartDashboard.putNumber("getVelocityX()", ahrs.getVelocityX());
        // SmartDashboard.putNumber("getVelocityY()", ahrs.getVelocityY());
        // SmartDashboard.putNumber("getVelocityZ()", ahrs.getVelocityZ());

        SmartDashboard.putNumber("Pitch", pigeon.getPitch());
        SmartDashboard.putNumber("Roll", pigeon.getRoll());
        SmartDashboard.putNumber("Yaw", pigeon.getYaw());
        //SmartDashboard.putNumber("pigeon2 Angle", MathUtil.mod(getAngleRad(), 2 * Math.PI));
    }

    public Rotation2d getAngleRad() {
        return Rotation2d.fromRadians(MathUtil.mod(pigeon.getYaw() * 2 * Math.PI / 360, Math.PI * 2));
    }

    public Rotation2d getAngleDeg() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    /*public double getAngle() {
        return ahrs.getAngleAdjustment();
    }*/

    public void zeroYaw() {
        pigeon.configMountPose(-90,0, 0 ,100);
        pigeon.setYaw(0);
        pigeon.setAccumZAngle(0);
    }

    /*public double getVelocity() {
        return Math.sqrt(Math.pow(ahrs.getVelocityX(), 2) + Math.pow(ahrs.getVelocityY(), 2));
    }*/
}
