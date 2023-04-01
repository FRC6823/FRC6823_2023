package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperAngleSubsystem extends SubsystemBase{
    private final SparkMaxAbsoluteEncoder.Type kType;

    private CANSparkMax angleMotor;
    private SparkMaxAbsoluteEncoder encoder;
    private PIDController pid;
    private boolean mode; //true is position mode (default), false is velocity mode (driver controlled)
    private double setPoint;
    private double speed;
    private boolean disabled;
    
    public GripperAngleSubsystem () {
        angleMotor = new CANSparkMax(11, MotorType.kBrushed);
        angleMotor.restoreFactoryDefaults();
        kType = SparkMaxAbsoluteEncoder.Type.kDutyCycle;
        encoder = angleMotor.getAbsoluteEncoder(kType);
        mode = true;
        disabled = true;
        setPoint = getPosition();
        speed = 0;

        SendableRegistry.addLW(this, "Pulley");

        pid = new PIDController(7, 0, 0);

        //mode = true;
    }

    public void setMode(boolean mode)
    {
        this.mode = mode;
    }

    public void setSetPoint(double setPoint)
    {
        this.setPoint = setPoint;
        disabled = false;
    }

    public void setSpeed(double speed){
        this.speed = speed;
        disabled = false;
    }

    public double getPosition()
    {
        //if (encoder.getPosition() > 180) 
            //return 0;
        return encoder.getPosition();
    }

    public void disable(){
        disabled = true;
    }

    public void enable(){
        disabled = false;
    }

    public boolean isAtSetPoint(){
        return getPosition() < setPoint + 0.05 && getPosition() > setPoint - 0.05;
    }

    @Override
    public void periodic()
    {
        if (!disabled){
            if (mode){
                pid.setSetpoint(setPoint);
                angleMotor.set(pid.calculate(encoder.getPosition()));
                SmartDashboard.putNumber("Wrist SetPt", setPoint);
                SmartDashboard.putNumber("Wrist Encoder", encoder.getPosition());
            }
            else{
                angleMotor.set(speed);
                setPoint = getPosition();
            }
        }
        else{
            angleMotor.disable();
            setPoint = getPosition();
        }
    }
}

