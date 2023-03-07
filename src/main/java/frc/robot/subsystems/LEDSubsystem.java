package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.util.Constants;

public class LEDSubsystem {
    private Spark blinkin;

    public LEDSubsystem(int PWM){
        blinkin = new Spark(PWM);
        wildcats();
    }

    public void cone(){
        blinkin.set(0.69);
    }

    public void cube(){
        blinkin.set(0.91);
    }

    public void rainbow(){
        blinkin.set(-0.99);
    }

    public void green(){
        blinkin.set(0.71);
    }

    public void allianceColor(){
        if (Constants.isRed){
            blinkin.set(0.61);
        }
        else{
            blinkin.set(0.85);
        }
    }

    public void wildcats(){
        blinkin.set(0.53);
    }
}
