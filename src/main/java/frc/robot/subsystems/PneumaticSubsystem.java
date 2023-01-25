package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PneumaticSubsystem {
    private Compressor compressor;
    private DoubleSolenoid intakeSolenoid;
    boolean state;

    public PneumaticSubsystem()
    {
        compressor = new Compressor(0, PneumaticsModuleType.REVPH);
        intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 29, 30);
        state = true;
    }

    public void periodic()
    {
        compressor.enableDigital();
    }

    public void togglePneumaticState()
    {
        state = !state;
    }

    public void setIntakeState(boolean openState)
    {
        if (openState){
            intakeSolenoid.set(Value.kForward);
        }
        else{
            intakeSolenoid.set(Value.kReverse);
        }
    }

}
