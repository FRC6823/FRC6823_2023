package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class PulleySubsystem extends SubsystemBase{
    private final SparkMaxAlternateEncoder.Type kType;
    private final int kCPR = 8192;

    private double setPoint;
    private CANSparkMax angleMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr; //heavily "inspired" by Rev example code
    //private boolean mode; //true is position mode (default), false is velocity mode (driver controlled)                                                                              z 
    private boolean mode;
    private double speed;
    
    public PulleySubsystem () {
        angleMotor = new CANSparkMax(10, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        kType = SparkMaxAlternateEncoder.Type.kQuadrature;
        encoder = angleMotor.getAlternateEncoder(kType, kCPR);
        pidController = angleMotor.getPIDController();
        pidController.setFeedbackDevice(encoder);
        angleMotor.setIdleMode(IdleMode.kBrake);
        SendableRegistry.addLW(this, "Pulley");
        speed = 0;
        setPoint = 0;

        // PID coefficients
        kP = 150; //5e-5
        kI = 1e-5; //1e-4
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        //maxRPM = 5700;

        //maxVel = 2000; // values in rpm
        //maxAcc = 1500;

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        //int smartMotionSlot = 0;
        //pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        //pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        //pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        //pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
        
        mode = true;
    }


    //public void setMode(boolean mode)
    //{
        //this.mode = mode;
    //}

    public void setSpeed(double speed)
    {
        this.speed = speed;
    }

    public void setSetPoint(double setPoint)
    {
        this.setPoint = setPoint;
    }

    public void setMode(boolean mode){
        this.mode = mode;
    }

    public double getPosition()
    {
        return encoder.getPosition();
    }

    @Override
    public void periodic()
    {
        if(mode) {
            setPoint = Math.min(setPoint, Constants.ELEVATOR_MAX);
            setPoint = Math.max(setPoint , Constants.ELEVATOR_MIN);
            SmartDashboard.putNumber("Pulley Position", setPoint);
            SmartDashboard.putNumber("Relative Encoder", encoder.getPosition());
            //pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
        } else {
            if (getPosition() >= Constants.ELEVATOR_MAX){
                speed = Math.min(speed, 0);
            }
            if (getPosition() <= Constants.ELEVATOR_MIN){
                speed = Math.max(speed, 0);
            }
            setPoint = getPosition();
            //angleMotor.set(speed);
        }
    }
}
