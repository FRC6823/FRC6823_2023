package frc.robot;

//import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.sensors.PigeonIMU;

public class Pigeon2Handler {
    private PigeonIMU pigeon;
    private double initialAngle = Math.PI/2.0;

    public double getInitialAngle() {
        return initialAngle;
    }

    public void setInitialAngle() {
        initialAngle = getAngleRad();
    }

    public PigeonIMU getAhrs() {
        return pigeon;
    }

    public Pigeon2Handler() {
        try {
            /***********************************************************************
             * pigeon2-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
             * http://pigeon2-mxp.kauailabs.com/guidance/selecting-an-interface.
             ***********************************************************************/
            pigeon = new PigeonIMU(18);
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

        SmartDashboard.putNumber("pigeon2 Angle", MathUtil.mod(getAngleRad(), 2 * Math.PI));
    }

    public double getAngleRad() {
        return MathUtil.mod(((pigeon.getYaw()) * 2 * Math.PI) / 360, Math.PI * 2);
    }

    /*public double getAngle() {
        return ahrs.getAngleAdjustment();
    }*/

    public void zeroYaw() {
        pigeon.setYaw(90);
    }

    /*public double getVelocity() {
        return Math.sqrt(Math.pow(ahrs.getVelocityX(), 2) + Math.pow(ahrs.getVelocityY(), 2));
    }*/
}
