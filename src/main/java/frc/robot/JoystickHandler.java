package frc.robot;

//import com.kauailabs.pigeon2.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
//import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.MathUtil;

public class JoystickHandler {

    private Joystick joystick;
    private double deadZone;

    public JoystickHandler(int joyNum) {
        this.joystick = new Joystick(joyNum); //Joystick is on port 3;
	}

	public double getRawAxis0() {
        return joystick.getRawAxis(0);
    }

    public double getRawAxis1() {
        return joystick.getRawAxis(1);
    }

    public double getRawAxis2() {
        return joystick.getRawAxis(2);
    }

    public double getRawAxis3() {
        return joystick.getRawAxis(3);
    }

    public double getRawAxis4() {
        return joystick.getRawAxis(4);
    }

    public double getRawAxis5() {
        return joystick.getRawAxis(5);
    }

    public double getRawAxis6() {
        return joystick.getRawAxis(6);
    }

    public boolean isFullThrottle() {
        // -1 is forward, 1 is backward
        // If full forward, go full throttle.
        return getRawAxis2() < -.9;
    }

    //Dead zone provision for main joystick
    public double getAxis0() {
        return MathUtil.clipToZero(getRawAxis0(), deadZone);
    }

    public double getAxis1() {
        return MathUtil.clipToZero(getRawAxis1(), deadZone);
    }

    public double getAxis2() {
        return MathUtil.clipToZero(getRawAxis2(), deadZone);
    }

    public double getAxis3() {
        return MathUtil.clipToZero(getRawAxis3(), deadZone);
    }

    public double getAxis4() {
        return MathUtil.clipToZero(getRawAxis4(), deadZone);
    }

    public double getAxis5() {
        return MathUtil.clipToZero(getRawAxis5(), deadZone);
    }

    public double getAxis6() {
        return MathUtil.clipToZero(getRawAxis6(), deadZone);
    }

    public Joystick joystick() {
        return joystick;
    }

    public JoystickButton button(int buttonNumber) {
        return new JoystickButton(joystick, buttonNumber);
    }

    public boolean isJoystickInUse() { //Is main joystick active at all
        return getAxis0() != 0 || getAxis1() != 0 || getAxis5() != 0;
    }

    public void updateDeadZone(){
        if (!Preferences.containsKey("Dead Zone"))
            Preferences.setDouble("Dead Zone", 0.05);
        deadZone = Preferences.getDouble("Dead Zone", 0.05);
    }
}