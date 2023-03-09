package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperAngleSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.PulleySubsystem;

public class WaitUntilPose extends CommandBase{
    private boolean inPose;
    private LiftSubsystem lift;
    private PulleySubsystem pulley;
    private GripperAngleSubsystem gripperAngle;

    public WaitUntilPose(LiftSubsystem lift, PulleySubsystem pulley, GripperAngleSubsystem gripperAngle){
        this.lift = lift;
        this.pulley = pulley;
        this.gripperAngle = gripperAngle;
    }

    public void initialize(){
        inPose = false;
    }

    public void execute(){
        inPose = lift.isAtSetPoint() && pulley.isAtSetPoint() && gripperAngle.isAtSetPoint();
    }

    public boolean isFinished(){
        return inPose;
    }
}
