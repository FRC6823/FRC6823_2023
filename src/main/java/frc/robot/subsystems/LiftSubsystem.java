package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase{
    private CANSparkMax angleMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr; //heavily "inspired" by Rev example code
    private boolean mode; //true is position mode (default), false is velocity mode (driver controlled)
    private double setPoint;
    private double speed;
    private boolean disabled;
    
    public LiftSubsystem () {
        angleMotor = new CANSparkMax(9, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        pidController = angleMotor.getPIDController();
        encoder = angleMotor.getEncoder();
        angleMotor.setIdleMode(IdleMode.kBrake);
        SendableRegistry.addLW(this, "Lift Extension");
        setPoint = getPosition();
        speed = 0;
        disabled = false;
        // PID coefficients
        kP = .1; //5e-5
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
        
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

    public void setSpeed(double speed)
    {
        this.speed = speed;
    }

    public double getPosition()
    {
        return encoder.getPosition();
    }

    public void disable(){
        disabled = true;
    }

    public void enable(){
        disabled = false;
    }

    public boolean isAtSetPoint(){
        return getPosition() < setPoint + 0.5 && getPosition() > setPoint - 0.5;
    }

    @Override
    public void periodic()
    {
        if(!disabled){
            if(mode) {
                //setPoint = Math.min(setPoint, Constants.EXTENSION_MIN);
                //setPoint = Math.max(setPoint , Constants.EXTENSION_MAX);
                SmartDashboard.putNumber("Lift Extension", setPoint);
                SmartDashboard.putNumber("Lift Encoder", getPosition());
                pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
            } else {
                /*if (getPosition() >= Constants.EXTENSION_MIN){
                    speed = Math.min(speed, 0);
                }
                if (getPosition() <= Constants.EXTENSION_MAX){
                    speed = Math.max(speed, 0);
                }*/
                setPoint = getPosition();
                angleMotor.set(speed);
            }
        }
        else {
            angleMotor.disable();
            setPoint = getPosition();
        }
    }
}
