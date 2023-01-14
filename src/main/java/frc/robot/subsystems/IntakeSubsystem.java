package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
//import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import java.util.HashSet;

public class IntakeSubsystem extends SubsystemBase {

    // private final double P = .04;
    // private final double I = .00001;
    private CANSparkMax angleMotor;
    private CANSparkMax intakeMotor;
    //private DutyCycleEncoder angleEncoder;
    private double inTakePower;
    private double anglePower;
    // private double margin;
    // private double downPos;
    // private double pidPower;
    private SimpleWidget intakeWidget;
    private SimpleWidget angleWidget;
    //private PIDController pid;

    public IntakeSubsystem() {
        this.angleMotor = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.intakeMotor = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
        //this.angleEncoder = new DutyCycleEncoder(2);
        intakeWidget = Shuffleboard.getTab("Preferences").addPersistent("intakePercent", 0.433)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1));
        angleWidget = Shuffleboard.getTab("Preferences").addPersistent("hammerPercent", 0.433)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1));
        // margin = 0.1;
        // downPos = 0.32;

        // pid = new PIDController(P, I, 0);

        SendableRegistry.addChild(this, angleMotor);
        SendableRegistry.addChild(this, intakeMotor);
        SendableRegistry.addLW(this, "Intake");

    }

    public CommandBase backIntake() {
        return this.runOnce(() -> 
        intakeMotor.set(-inTakePower * 0.5)
        );
    }

    public HashSet<Subsystem> angle() {
        // while(Math.abs(angleEncoder.getAbsolutePosition() - downPos) > margin){
        //     pidPower = pid.calculate(angleEncoder.getAbsolutePosition(), downPos);
        //     angleMotor.set(-pidPower);
        // }
        HashSet<Subsystem> tree = new HashSet<Subsystem>();
        tree.add(this);
        angleMotor.set(-1.5 * anglePower);
        return tree;
    }

    public CommandBase intake() {
        return this.runOnce(() ->
        intakeMotor.set(inTakePower)
        );
    }

    public HashSet<Subsystem> backAngle() {
        HashSet<Subsystem> tree = new HashSet<Subsystem>();
        tree.add(this);
        angleMotor.set(anglePower);
        return tree;
    }

    public void backAngle(double power){
        angleMotor.set(power);
    }

    public CommandBase stopIntake() {
        return this.runOnce(() -> intakeMotor.set(0));
    }

    public void stopAngle() {
        angleMotor.set(0);
    }

    @Override
    public void periodic() {
        inTakePower = intakeWidget.getEntry().getDouble(-2);
        anglePower = angleWidget.getEntry().getDouble(-2);
        //SmartDashboard.putNumber("Intake Angle", angleEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Intake Speed", intakeMotor.getEncoder().getVelocity());
    }
}