package frc.robot.subsystems;

import java.util.Map;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {

    private CANSparkMax leftLiftMotor;
    private CANSparkMax rightLiftMotor;
    private SimpleWidget leftPowWidget;
    private SimpleWidget rightPowWidget;
    private double leftLiftPower;
    private double rightLiftPower;

    public LiftSubsystem() {
        this.leftLiftMotor = new CANSparkMax(16, CANSparkMaxLowLevel.MotorType.kBrushed);
        this.rightLiftMotor = new CANSparkMax(17, CANSparkMaxLowLevel.MotorType.kBrushed);
        leftPowWidget = Shuffleboard.getTab("Preferences").add("Left Lift Power", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1));
        rightPowWidget = leftPowWidget = Shuffleboard.getTab("Preferences").add("Right Lift Power", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1));
        periodic();

        SendableRegistry.addChild(this, leftLiftMotor);
        SendableRegistry.addChild(this, rightLiftMotor);
        SendableRegistry.addLW(this, "Lift");

    }

    public void liftDown() {
        leftLiftMotor.set(-leftLiftPower);
        rightLiftMotor.set(-rightLiftPower);
    }
    
    public void liftUp() {
        leftLiftMotor.set(leftLiftPower);
        rightLiftMotor.set(rightLiftPower);
    }

    public void leftUp(){
        leftLiftMotor.set(leftLiftPower);
    }

    public void leftDown(){
        leftLiftMotor.set(-leftLiftPower);
    }

    public void rightUp(){
        rightLiftMotor.set(rightLiftPower);
    }

    public void rightDown(){
        rightLiftMotor.set(-rightLiftPower);
    }

    public void liftStop() {
        leftLiftMotor.set(0);
        rightLiftMotor.set(0);
    }

    @Override
    public void periodic(){
        leftLiftPower = leftPowWidget.getEntry().getDouble(1);
        rightLiftPower = rightPowWidget.getEntry().getDouble(1);
    }
}
