package frc.robot;

import java.util.*;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperAngleSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.PulleySubsystem;
import frc.robot.util.Constants;

public class PositionHandler extends CommandBase{
    private ArrayList<double[]> positions;  //[0] - lift extension position
                                            //[1] - lift angle
                                            //[2] - gripper angle
    private int index;
    public boolean calibrate; //default - not calibrating/in competition (false)

    private LiftSubsystem liftSubsystem;
    private PulleySubsystem pulleySubsystem;
    private GripperAngleSubsystem gripperAngleSubsystem;

    public PositionHandler (LiftSubsystem liftSubsystem, PulleySubsystem pulleySubsystem, GripperAngleSubsystem gripperAngleSubsystem){
        index = 0;
        positions = new ArrayList<double[]>();
        positions.add(Constants.startPose);
        positions.add(Constants.highScorePose);
        positions.add(Constants.lowScorePose);
        positions.add(Constants.transportPose);
        positions.add(Constants.pickupPose);
        positions.add(Constants.floorPose);

        this.liftSubsystem = liftSubsystem;
        this.pulleySubsystem = pulleySubsystem;
        this.gripperAngleSubsystem = gripperAngleSubsystem;

        SendableRegistry.addLW(this, "Pose Handler");

        calibrate = true;
    }

    public void increaseIndex() {
        if (index <= positions.size() - 2)
            index++;
    }

    public void decreaseIndex() {
        if (index >= 1)
            index--;
    }

    public void setState(boolean state){
        calibrate = state;
    }

    public void capturePose() {
        if (calibrate) {
            double[] temp = new double[3];
            temp[0] = liftSubsystem.getPosition();
            temp[1] = pulleySubsystem.getPosition();
            temp[2] = gripperAngleSubsystem.getPosition(); //will be gripper angle
            positions.add(index, temp);
        }
    }

    public void setPose() {
        liftSubsystem.setSetPoint(positions.get(index)[0]);
        pulleySubsystem.setSetPoint(positions.get(index)[1]);
        gripperAngleSubsystem.setSetPoint(positions.get(index)[2]);

        SmartDashboard.putNumber("Index", index);
    }

    public void setPose(double[] pose) {
        liftSubsystem.setSetPoint(pose[0]);
        pulleySubsystem.setSetPoint(pose[1]);
        gripperAngleSubsystem.setSetPoint(positions.get(index)[2]);
    }

    public void setPose(int i){
        index = i;
        liftSubsystem.setSetPoint(positions.get(i)[0]);
        pulleySubsystem.setSetPoint(positions.get(i)[1]);
        gripperAngleSubsystem.setSetPoint(positions.get(i)[2]);

        SmartDashboard.putNumber("Index", i);
    }
}
