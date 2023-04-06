package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase{
    
    //All items for the left limelight will be prefixed with "l"
    //All items for the right limelight will be prefixed with "r"

    private NetworkTable leftTable;
    private NetworkTable rightTable;
    private NetworkTableEntry ltx, rtx;
    private NetworkTableEntry lty, rty;
    //private NetworkTableEntry ltv, rtv;
    private NetworkTableEntry lb_t, rb_t; //Botpose relative to Target
    private NetworkTableEntry lid, rid;

    public LimeLightSubsystem(){
        leftTable = NetworkTableInstance.getDefault().getTable("limelight-left");
        rightTable = NetworkTableInstance.getDefault().getTable("limelight-right");
        ltx = leftTable.getEntry("tx");
        lty = leftTable.getEntry("ty");
        //ltv = leftTable.getEntry("tv");
        lb_t = leftTable.getEntry("botpose_targetspace");
        lid = leftTable.getEntry("tid");

        rtx = rightTable.getEntry("tx");
        rty = rightTable.getEntry("ty");
        //rtv = rightTable.getEntry("tv");
        rb_t = rightTable.getEntry("botpose_targetspace");
        rid = rightTable.getEntry("tid");

        SendableRegistry.addLW(this, "LimeLight");
    }

    //Pipeline management methods
    public void lSetPipeline(int pipeline){
        leftTable.getEntry("pipeline").setNumber(pipeline);
    }

    public void rSetPipeline(int pipeline){
        rightTable.getEntry("pipeline").setNumber(pipeline);
    }

    public int lGetPipeline(){
        return (int) leftTable.getEntry("getpipe").getDouble(0);
    }

    public int rGetPipeline(){
        return (int) rightTable.getEntry("getpipe").getDouble(0);
    }

    //General info methods
    public double lGetTx() {
        return ltx.getDouble(0);
    }
    public double lGetTy() {
        return lty.getDouble(0);
    }
    public double lGetId() {
        return lid.getDouble(0);
    }

    public double rGetTx() {
        return rtx.getDouble(0);
    }
    public double rGetTy() {
        return rty.getDouble(0);
    }
    public double rGetId() {
        return rid.getDouble(0);
    }

    //3d tracking methods
    public double[] lGetTargetSpacePose() {
        return lb_t.getDoubleArray(new double[]{0,0,0,0,0,0});
    }
    
    public double[] rGetTargetSpacePose() {
        return rb_t.getDoubleArray(new double[]{0,0,0,0,0,0});
    }

    public boolean lHasValidTarget(){
        return lGetTargetSpacePose()[1] != 0;  
    }

    public boolean rHasValidTarget(){
        return rGetTargetSpacePose()[1] != 0;  
    }

    public double lGet3dTX() {
        return lGetTargetSpacePose()[0];
    }
    public double lGet3dTZ() {
        return lGetTargetSpacePose()[2];
    }

    public double rGet3dTX() {
        return rGetTargetSpacePose()[0];
    }
    public double rGet3dTZ() {
        return rGetTargetSpacePose()[2];
    }

    public double rGet3dRY() {
        return rGetTargetSpacePose()[4];
    }
    public double lGet3dRY() {
        return lGetTargetSpacePose()[4];
    }

    public void periodic() {
        SmartDashboard.putNumber("Right TX", rGet3dTX());
        SmartDashboard.putNumber("Left TX", lGet3dTX());
        SmartDashboard.putNumber("Right TZ", rGet3dTZ());
        SmartDashboard.putNumber("Left TZ", lGet3dTZ());
        SmartDashboard.putNumber("Right RY", rGet3dRY());
        SmartDashboard.putNumber("Left RY", lGet3dRY());
    }
}