package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.JoystickHandler;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ThrottleSubsystem extends SubsystemBase {

    private CANSparkMax motor, motor2;
    double speed;
    JoystickHandler joy;

    public ThrottleSubsystem(int id, JoystickHandler joy) {
        motor = new CANSparkMax(id, MotorType.kBrushless);
        motor2 = new CANSparkMax(14, MotorType.kBrushless);
        speed = 01;
        this.joy = joy;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
        if (speed > .05) {
            speed = 0;
        }
    }

    public void periodic() {
        setSpeed((joy.getAxis6() + 1) / 2);
        motor.set(-speed);
        motor2.set(speed);

    }

}
