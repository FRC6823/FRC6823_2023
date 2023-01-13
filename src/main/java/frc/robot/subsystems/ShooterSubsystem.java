package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final double P = .02; // was 0.04
    private final double I = .0004;
    private final double D = 0;
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    private CANSparkMax angleMotor;
    private CANSparkMax loadMotor;
    // private AnalogInput angleEncoder;
    // private int offset;
    private int velocityLeft;
    private int velocityRight;
    // private int test;
    // private DigitalInput frontLimit;
    // private DigitalInput backLimit;
    // private PIDController speedController;
    // private SparkMaxAlternateEncoder.Type altEncoderType =
    // SparkMaxAlternateEncoder.Type.kQuadrature;
    // private Encoder encoder;
    private int shooterRPMLeft;
    private int shooterRPMRight;
    private double shooterAnglePercentBack;
    private double shooterAnglePercentForward;
    private double loadPercent;
    private double ratio;
    private double percent;
    private DutyCycleEncoder encoder;
    private PIDController pidController;
    private SimpleWidget loadWidget;
    private SimpleWidget RPMRatio;
    private SimpleWidget RPMPercent;

    public ShooterSubsystem() {
        this.leftMotor = new TalonFX(11);
        this.rightMotor = new TalonFX(12);
        this.angleMotor = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushed);
        this.loadMotor = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);
        // this.angleEncoder = new AnalogInput(0);
        // this.speedController = new PIDController(0.0001, 0, 0);
        this.encoder = new DutyCycleEncoder(0);
        this.pidController = new PIDController(P, I, D);
        pidController.setTolerance(20);
        velocityLeft = 0;
        velocityRight = 0;
        loadWidget = Shuffleboard.getTab("Preferences").add("LoadRate", 0.6)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -1, "max", 1));
        SendableRegistry.addLW(this, "Shooter");
        RPMRatio = Shuffleboard.getTab("Preferences").add("shooterRPMOffset", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withProperties(Map.of("min", -1, "max", 1));
        RPMPercent = Shuffleboard.getTab("Preferences").add("shooterRPMPercent", 0.69)
                .withWidget(BuiltInWidgets.kTextView)
                .withProperties(Map.of("min", 0, "max", 1));
    }

    public void shoot(int rpm) {
        velocityLeft = rpm; // rpm * 2048 units/rotation * intervals of 100 ms per minute
        velocityRight = rpm; // rpm * 2048 units/rotation * intervals of 100 ms per minute
        velocityLeft *= -1;
        velocityRight *= -1;
        leftMotor.set(ControlMode.Velocity, -velocityLeft); // velocity in encoder units per 100 ms
        rightMotor.set(ControlMode.Velocity, velocityRight);
        //SmartDashboard.putNumber("velocityLeft target", velocityLeft);
        //SmartDashboard.putNumber("velocityRight target", velocityRight);
    }

    public void shoot(int rpmLeft, int rpmRight) {
        velocityLeft = rpmLeft; // rpm * 2048 units/rotation * intervals of 100 ms per minute
        velocityRight = rpmRight; // rpm * 2048 units/rotation * intervals of 100 ms per minute
        // velocityLeft *= -1;
        // velocityRight *= -1;
        leftMotor.set(ControlMode.Velocity, -velocityLeft); // velocity in encoder units per 100 ms
        rightMotor.set(ControlMode.Velocity, velocityRight);
        //SmartDashboard.putNumber("velocityLeft target", velocityLeft);
        //SmartDashboard.putNumber("velocityRight target", velocityRight);
        // leftMotor.getSelectedSensorVelocity();
        // test += 100;
    }

    public void shootPower(double power) {
        leftMotor.set(ControlMode.PercentOutput, -power); //velocity in encoder units per 100 ms
        rightMotor.set(ControlMode.PercentOutput, power);
    }

    public void prep(double load) {
        loadMotor.set(load);
    }

    public void loadStop() {
        loadMotor.set(0);
    }

    public void backLoad(double power) {
        loadMotor.set(-power);
    }

    public int getShooterRPMLeft() {
        return shooterRPMLeft;
    }

    public int getShooterRPMRight() {
        return shooterRPMRight;
    }

    public double getShooterAnglePercentBack() {
        return shooterAnglePercentBack;
    }

    public double getShooterAnglePercentForward() {
        return shooterAnglePercentForward;
    }

    public double getLoadPercent() {
        return loadPercent;
    }

    public void shootStop() {
        leftMotor.set(ControlMode.PercentOutput, 0);
        rightMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setShooterAngle(double angle) {
        double currentEncoderValue = (encoder.get() - 0.383) * 360;
        double setpoint = -Math.abs(angle);
        pidController.setSetpoint(setpoint);
        double pidOut = pidController.calculate(currentEncoderValue, setpoint);
        angleMotor.set(-pidOut);
        //SmartDashboard.putBoolean("settingSA", true);
        //SmartDashboard.putNumber("shooterPIDout", pidOut);
    }

    @Override
    public void periodic() {
        percent = RPMPercent.getEntry().getInteger(2) * 2;
        ratio = RPMRatio.getEntry().getInteger(2) * 2;

        if(ratio >= 0){
            shooterRPMLeft = (int)(6000 * percent);
            shooterRPMRight = (int)((1 - ratio) * 6000 * percent);
        }else{
            ratio *= -1;
            shooterRPMLeft = (int)((1 - ratio) * 6000 * percent);
            shooterRPMRight = (int)(6000 * percent);
        }
        
        loadPercent = loadWidget.getEntry().getDouble(-1);

        // SmartDashboard.putNumber("Left shoot rpm",
        // leftMotor.getSelectedSensorVelocity() / 600 / 2048);
        // SmartDashboard.putNumber("Right shoot rpm",
        // rightMotor.getSelectedSensorVelocity() / 600 / 2048);
        SmartDashboard.putNumber("Left shoot rpm", leftMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Right shoot rpm", rightMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("shooter angle", encoder.get());
    }

}