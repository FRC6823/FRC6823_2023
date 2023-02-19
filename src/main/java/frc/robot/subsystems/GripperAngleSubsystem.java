package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperAngleSubsystem extends SubsystemBase{
    /*private CANSparkMax angleMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr; //heavily "inspired" by Rev example code
    private boolean mode; //true is position mode (default), false is velocity mode (driver controlled)
    private double setPoint;
    
    public GripperAngleSubsystem () {
        angleMotor = new CANSparkMax(0, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        pidController = angleMotor.getPIDController();
        encoder = angleMotor.getEncoder();

        // PID coefficients
        kP = 5e-5; 
        kI = 1e-6;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        maxVel = 2000; // values in rpm
        maxAcc = 1500;

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        int smartMotionSlot = 0;
        pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
        
        mode = true;
    }

    public void setMode(boolean mode)
    {
        this.mode = mode;
    }

    public void setSetPoint(double setPoint)
    {
        this.setPoint = setPoint;
    }

    public double getPosition()
    {
        return encoder.getPosition();
    }

    @Override
    public void periodic()
    {
        if(mode) {
            pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
          } else {
            pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
          }
    }*/
}

