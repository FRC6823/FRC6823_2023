package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase{
    
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;
    private NetworkTableEntry b_t; //Botpose relative to Target
    //private NetworkTableEntry B_F; //Botpose relative to Field
    private NetworkTableEntry id;

    public LimeLightSubsystem(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        b_t = table.getEntry("botpose_targetspace");
        id = table.getEntry("tid");

        SendableRegistry.addLW(this, "LimeLight");
    }

    public void setPipeline(int pipeline){
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public int getPipeline(){
        return (int) table.getEntry("getpipe").getDouble(0);
    }

    public double getTx() {
        return tx.getDouble(0);
    }
    public double getTy() {
        return ty.getDouble(0);
    }
    public double getTa() {
        return ta.getDouble(0);
    }
    public double getTv(){
        return tv.getDouble(0);
    }
    public double getId() {
        return id.getDouble(0);
    }

    public double[] getTargetSpacePose() {
        return b_t.getDoubleArray(new double[]{0,0,0,0,0,0});
    }

    public boolean hasValidTarget(){
        return table.getEntry("tv").getDouble(0) == 1;  
    }

    public double get3dTX() {
        return getTargetSpacePose()[0];
    }

    public double get3dRY() {
        return getTargetSpacePose()[4];
    }

    public double get3dTZ() {
        return getTargetSpacePose()[2];
    }

    public void periodic() {
        SmartDashboard.putNumber("pipeline", table.getEntry("pipeline").getDouble(0));
        SmartDashboard.putNumber("target?", getTv());
    }
}