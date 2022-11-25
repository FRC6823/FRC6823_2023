package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

import com.kauailabs.navx.frc.AHRS;

public class NavXHandler {
    private AHRS ahrs;
    private double initialAngle;

    public double getInitialAngle() {
        return initialAngle;
    }

    public void setInitialAngle() {
        initialAngle = getAngleRad();
    }

    public AHRS getAhrs() {
        return ahrs;
    }

    public NavXHandler() {
        try {
            /***********************************************************************
             * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
             * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
             ***********************************************************************/
            ahrs = new AHRS(SerialPort.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
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

        SmartDashboard.putNumber("Pitch", ahrs.getPitch());
        SmartDashboard.putNumber("Roll", ahrs.getRoll());
        SmartDashboard.putNumber("Yaw", ahrs.getYaw());

        SmartDashboard.putNumber("NavX Angle", MathUtil.mod(getAngleRad(), 2 * Math.PI));
    }

    public double getAngleRad() {
        return MathUtil.mod(ahrs.getAngle() * 2 * Math.PI / 360, Math.PI * 2);
    }

    public double getAngle() {
        return ahrs.getAngleAdjustment();
    }

    public void zeroYaw() {
        ahrs.reset();
    }

    public double getVelocity() {
        return Math.sqrt(Math.pow(ahrs.getVelocityX(), 2) + Math.pow(ahrs.getVelocityY(), 2));
    }
}
