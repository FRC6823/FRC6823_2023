package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase{
    private Compressor compressor;
    private DoubleSolenoid intakeSolenoid;
    private boolean state;

    public PneumaticSubsystem()
    {
        compressor = new Compressor(29, PneumaticsModuleType.REVPH);
        intakeSolenoid = new DoubleSolenoid(29, PneumaticsModuleType.REVPH, 0, 1);
        intakeSolenoid.set(Value.kForward);
        SendableRegistry.addLW(this, "pressure");
        state = true;
        //openSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
        //closeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
    }

    public void periodic()
    {
        compressor.enableDigital();

        if (state){
            intakeSolenoid.set(Value.kForward);
        }
        else{ 
            intakeSolenoid.set(Value.kReverse);
        }
    }
    
    public void togglePneumaticState(){
        state = !state;
    }
}
