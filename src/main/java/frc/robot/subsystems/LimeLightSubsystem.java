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
    private NetworkTableEntry b_t; //Botpose relative to Target
    //private NetworkTableEntry B_F; //Botpose relative to Field
    private NetworkTableEntry id;
    private double[] lastKnownX_Z_Tx;

    public LimeLightSubsystem(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        //ta = table.getEntry("ta");
        // b_t = table.getEntry("camPose");
        b_t = table.getEntry("botpose_targetspace");
        //id = table.getEntry("tid");
        //lastKnownX_Z_Tx = new double[3];
        SendableRegistry.addLW(this, "LimeLight");
        lastKnownX_Z_Tx = new double[]{0,0,0};
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
    public double getId() {
        return id.getDouble(0);
    }

    public double[] getB_T() {
        return b_t.getDoubleArray(new double[]{0,0,0,0,0,0});
    }

    public boolean hasValidTarget(){
        return table.getEntry("tv").getDouble(0) == 1;  
    }


    public double get3dTX() {
        return getB_T()[0];
    }

    public double get3dRY() {
        return getB_T()[4];
    }

    public void periodic() {
        //SmartDashboard.putNumber("tx", tx.getDouble(0));
        SmartDashboard.putNumber("tx", tx.getDouble(0.0));
        SmartDashboard.putNumber("ty", ty.getDouble(0.0));
        SmartDashboard.putNumber("TX", b_t.getDoubleArray(new double[]{0,0,0,0,0,0})[0]);
        //SmartDashboard.putNumber("ta", ta.getDouble(0));
        SmartDashboard.putNumber("pipeline", table.getEntry("pipeline").getDouble(0));
    }
}
