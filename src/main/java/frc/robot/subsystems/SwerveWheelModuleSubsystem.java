package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MathUtil;
import frc.robot.util.Constants;

public class SwerveWheelModuleSubsystem extends SubsystemBase {
    private final double P = .008;
    private final double I = .00001;

    private TalonFX angleMotor;
    private TalonFX speedMotor;
    private PIDController pidController;
    private CANCoder angleEncoder;
    //private boolean calibrateMode;
    private double encoderOffset;
    private String motorName;
    //private SimpleWidget speedLim;

    public SwerveWheelModuleSubsystem(int angleMotorChannel, int speedMotorChannel, int angleEncoderChannel,
            String motorName, double offset) {
        // We're using TalonFX motors on CAN.
        this.angleMotor = new TalonFX(angleMotorChannel);
        this.speedMotor = new TalonFX(speedMotorChannel);
        this.angleEncoder = new CANCoder(angleEncoderChannel); // CANCoder Encoder
        this.speedMotor.setNeutralMode(NeutralMode.Coast);
        this.motorName = motorName;
        this.pidController = new PIDController(P, I, 0); // This is the PID constant,
        // we're not using any
        // Integral/Derivative control but increasing the P value will make
        // the motors more aggressive to changing to angles.

        //angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        // pidController.setTolerance(20); //sets tolerance, shouldn't be needed.

        pidController.enableContinuousInput(0, 360); // This makes the PID controller
        // understand the fact that for
        // our setup, 360 degrees is the same as 0 since the wheel loops.

        SendableRegistry.addChild(this, angleMotor);
        SendableRegistry.addChild(this, speedMotor);
        SendableRegistry.addChild(this, angleEncoder);
        SendableRegistry.addLW(this, "Swerve Wheel Module");
        //speedLim = Shuffleboard.getTab("Preferences").addPersistent("Speed Lim", 0.5)
        //.withWidget(BuiltInWidgets.kNumberSlider);;
        encoderOffset = offset;
        resetSensor();
    }

    public void drive(double speed, double angle) {
        double currentEncoderValue = getPosition();
        int reverse = setAngle(angle, currentEncoderValue);
        setSpeed(speed * reverse);
        
        SmartDashboard.putNumber("Encoder " + motorName, getPosition());
        SmartDashboard.putNumber("Distance " + motorName, getDistance());
        SmartDashboard.putNumber("Rotation " + motorName, getPosition());
    }

    public int setAngle(double angle, double currentEncoderValue)
    {
        angle = MathUtil.mod(angle, 360); // ensure setpoint is on scale 0-360
        int reverse = 1;
        //angle += 90;

        //if (MathUtil.getCyclicalDistance(currentEncoderValue, angle, 360) > 90)
        //{
            //reverse = -1;
        //}
        
        double pidOut = -pidController.calculate(currentEncoderValue, angle);
        
        angleMotor.set(ControlMode.PercentOutput, pidOut);

        return reverse;
    }

    public void setSpeed(double speed)
    {
        speedMotor.set(ControlMode.PercentOutput, Math.min(speed, 0.5)); // sets motor speed //22150 units/100 ms at 12.4V
    }

    // this method outputs position of the encoder to the smartDashBoard, useful for
    // calibrating the encoder offsets
    public double getPosition() {
        return MathUtil.mod(angleEncoder.getAbsolutePosition() - encoderOffset, 360);
    }

    public double getDistance() {
        if (motorName.equals("BR") || motorName.equals("FR")) {
            return -(speedMotor.getSelectedSensorPosition() * Constants.WHEEL_CIRCUMFERENCE)/(2048 * Constants.L2_RATIO);
        }
        return (speedMotor.getSelectedSensorPosition() * Constants.WHEEL_CIRCUMFERENCE)/(2048 * Constants.L2_RATIO);
    }

    public void stop() {
        speedMotor.set(ControlMode.PercentOutput, 0);
        angleMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        //calibrateMode = calibrateState.getEntry().getBoolean(false);
    }

    public void coast(){
        speedMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void brake(){
        speedMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void resetSensor()
    {
        speedMotor.setSelectedSensorPosition(0);
    }

    //
    public SwerveModulePosition getSwerveModulePosition()
    {
        return new SwerveModulePosition(getDistance(), new Rotation2d(getPosition()));
    }
}
