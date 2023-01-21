package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
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
    private boolean calibrateMode;
    private double encoderOffset;
    private String motorName;
    private SimpleWidget calibrateState;

    public SwerveWheelModuleSubsystem(int angleMotorChannel, int speedMotorChannel, int angleEncoderChannel,
            String motorName, SimpleWidget calibrate, double offset) {
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
        calibrateState = calibrate;
        encoderOffset = offset;

    }

    // public void setZero(double offset) {
    //     encoderOffset = offset;
    // }

    // angle is a value between -1 to 1
    public void drive(double speed, double angle) {

        double currentEncoderValue = angleEncoder.getAbsolutePosition();

        // Optimal offset can be calculated here.
        angle *= 180;
        angle += encoderOffset;
        angle = MathUtil.mod(angle, 360); // ensure setpoint is on scale 0-360

        // if the setpoint is more than 90 degrees away form the current position, then
        // just reverse the speed
        // Set a variable to hold wheel position in degrees or one to hold
        // units/rotation
        
        // if (MathUtil.getCyclicalDistance(unitsToDegrees(currentEncoderValue), angle, 360) > 90) {
        //     speed *= -1;
        //     angle = (angle + 180) % 360;
        // }

        speedMotor.set(ControlMode.PercentOutput, speed); // sets motor speed //22150 units/100 ms at 12.4V
        //SmartDashboard.putNumber("Speed " + angleEncoderChannel, speed);

        // Sets angle motor to angle
       // pidController.setSetpoint(angle);
        double pidOut = -pidController.calculate(currentEncoderValue, angle);
        // pidOut *= 3000 * 4096 * 600; //pidOut is on [-1, 1], pidOut * 3000 (Max rpm)
        // * 4096 units/revolution * (600*100)ms/min

        //SmartDashboard.putNumber("Angle w/ offset", angle);
        //angle /= 360; // Angle position in rotations
        //SmartDashboard.putNumber("Position in revolutions", angle);
        //angle *= 26227; // Angle Position in encoder units
        //SmartDashboard.putNumber("Position[" + angleEncoderChannel + "]", angle);

        if (calibrateMode)
            angleMotor.set(ControlMode.PercentOutput, 0); // Sends new pidOut (in units/100 ms) to velocity control
        else
            angleMotor.set(ControlMode.PercentOutput, pidOut);

        SmartDashboard.putNumber("Encoder " + motorName, getPosition());
    }

    // this method outputs position of the encoder to the smartDashBoard, useful for
    // calibrating the encoder offsets
    public double getPosition() {
        return MathUtil.mod(angleEncoder.getAbsolutePosition() * 180 - encoderOffset, 360);
    }

    public void stop() {
        // pidController.setP(0);
        // pidController.setI(0);
        speedMotor.set(ControlMode.PercentOutput, 0);
        angleMotor.set(ControlMode.PercentOutput, 0);
    }

    public void restart() {
        // pidController.setP(P);
        // pidController.setP(I);
    }

    @Override
    public void periodic() {
        calibrateMode = calibrateState.getEntry().getBoolean(false);
    }

    // public double autoCali() {
    //     if (calibrateMode) {
    //         double offset = (angleEncoder.getAbsolutePosition() + 180) % 360;
    //         setZero(offset);
    //         return offset;
    //     } else {
    //         return 0;
    //     }
    // }

    // public double autoCaliZero(){
    //     setZero(0);
    //     return 0;
    // }

    // private double unitsToDegrees(double units){
    //     units = units / 26227 * 360 ;
    //     return MathUtil.mod(units, 360);
    // }

    public void coast(){
        speedMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void brake(){
        speedMotor.setNeutralMode(NeutralMode.Brake);
    }
}
