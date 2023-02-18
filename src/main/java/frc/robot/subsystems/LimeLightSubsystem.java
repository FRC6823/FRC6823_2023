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
    private double[] lastKnownXYZ;

    public LimeLightSubsystem(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        lastKnownXYZ = new double[3];
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

    public boolean hasValidTarget(){
        return table.getEntry("tv").getDouble(0) == 1;
    }


    public double[] getXYZ() {
        if (hasValidTarget()) {
            lastKnownXYZ[0] = table.getEntry("camtran").getDoubleArray(new double[] {0})[0];
            lastKnownXYZ[1] = table.getEntry("camtran").getDoubleArray(new double[] {0})[1];
            lastKnownXYZ[2] = table.getEntry("camtran").getDoubleArray(new double[] {0})[2];
        }
        return lastKnownXYZ;
    }

    public void periodic() {
        SmartDashboard.putNumber("tx", tx.getDouble(0));
        SmartDashboard.putNumber("ty", ty.getDouble(0));
        SmartDashboard.putNumber("ta", ta.getDouble(0));
        SmartDashboard.putNumber("pipeline", table.getEntry("pipeline").getDouble(0));
    }
}
