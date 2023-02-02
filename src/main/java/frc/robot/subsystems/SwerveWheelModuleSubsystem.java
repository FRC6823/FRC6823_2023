package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MathUtil;

public class SwerveWheelModuleSubsystem extends SubsystemBase {
    private final double P = .008;
    private final double I = .00001;

    private TalonFX angleMotor;
    private TalonFX speedMotor;
    private PIDController pidController;
    private CANCoder angleEncoder;
    private double encoderOffset;
    private String motorName;

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

        angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        // pidController.setTolerance(20); //sets tolerance, shouldn't be needed.

        pidController.enableContinuousInput(0, 360); // This makes the PID controller
        // understand the fact that for
        // our setup, 360 degrees is the same as 0 since the wheel loops.

        SendableRegistry.addChild(this, angleMotor);
        SendableRegistry.addChild(this, speedMotor);
        SendableRegistry.addChild(this, angleEncoder);
        SendableRegistry.addLW(this, "Swerve Wheel Module");
        encoderOffset = offset;

    }

    // angle is a value between -180 to 180
    public void drive(double speed, double angle) {
        double currentEncoderValue = angleEncoder.getAbsolutePosition();
        int reverse = setAngle(angle, currentEncoderValue);
        setSpeed(speed, reverse);

        SmartDashboard.putNumber("Encoder " + motorName, getPosition());
    }

    public int setAngle(double angle, double currentEncoderValue)
    {
        //angle = MathUtil.mod(angle, 360); // ensure setpoint is on scale 0-360
        int reverse = 1;
        angle += 90;

        // if the setpoint is more than 90 degrees away form the current position, then just reverse the speed
        if (MathUtil.getCyclicalDistance(currentEncoderValue, angle, 360) > 90) {
            reverse = -1;
            angle = (angle + 180) % 360;
        }
        
        double pidOut = -pidController.calculate(currentEncoderValue, angle);
        
        angleMotor.set(ControlMode.PercentOutput, pidOut);

        return reverse;
    }

    public void setSpeed(double speed, double reverse)
    {
        speedMotor.set(ControlMode.PercentOutput, speed * reverse);
    }

    // this method outputs position of the encoder to the smartDashBoard, useful for
    // calibrating the encoder offsets
    public double getPosition() {
        return MathUtil.mod(angleEncoder.getAbsolutePosition() * 180 - encoderOffset, 360);
    }

    public void stop() {
        speedMotor.set(ControlMode.PercentOutput, 0);
        angleMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {

    }

    public void coast(){
        speedMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void brake(){
        speedMotor.setNeutralMode(NeutralMode.Brake);
    }
}
