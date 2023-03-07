package frc.robot;

import java.util.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperAngleSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.PulleySubsystem;

public class PositionHandler extends CommandBase{
    private ArrayList<double[]> positions;  //[0] - lift extension position
                                            //[1] - lift angle
                                            //[2] - gripper angle
    private int index;
    public boolean calibrate; //default - not calibrating/in competition (false)

    private LiftSubsystem liftSubsystem;
    private PulleySubsystem pulleySubsystem;
    //private GripperAngleSubsystem gripperAngleSubsystem;

    public PositionHandler (LiftSubsystem liftSubsystem, PulleySubsystem pulleySubsystem, GripperAngleSubsystem gripperAngleSubsystem){
        index = 0;
        positions = new ArrayList<double[]>();
        positions.add(0, new double[] {0,0,0});

        this.liftSubsystem = liftSubsystem;
        this.pulleySubsystem = pulleySubsystem;
        //this.gripperAngleSubsystem = gripperAngleSubsystem;

        calibrate = true;
    }

    public void increaseIndex() {
        index++;
        index %= positions.size();
    }

    public void decreaseIndex() {
        if (index >= 1)
            index--;
        else
            index = positions.size();
        index %= positions.size();
    }

    public void setState(boolean state){
        calibrate = state;
    }

    public void capturePose() {
        if (calibrate) {
            double[] temp = new double[3];
            temp[0] = liftSubsystem.getPosition();
            temp[1] = pulleySubsystem.getPosition();
            //temp[2] = gripperAngleSubsystem.getPosition(); //will be gripper angle
            positions.add(index, temp);
        }
    }

    public void setPose() {
        liftSubsystem.setSetPoint(positions.get(index)[0]);
        pulleySubsystem.setSetPoint(positions.get(index)[1]);
    }

    public void setPose(double[] pose) {
        liftSubsystem.setSetPoint(pose[0]);
        pulleySubsystem.setSetPoint(pose[1]);
    }
}
