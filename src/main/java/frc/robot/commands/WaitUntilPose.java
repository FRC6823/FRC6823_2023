package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.PulleySubsystem;

public class WaitUntilPose extends CommandBase{
    private boolean inPose;
    private LiftSubsystem lift;
    private PulleySubsystem pulley;

    public WaitUntilPose(LiftSubsystem lift, PulleySubsystem pulley){
        this.lift = lift;
        this.pulley = pulley;
    }

    public void initialize(){
        inPose = false;
    }

    public void execute(){
        inPose = lift.isAtSetPoint() && pulley.isAtSetPoint();
    }

    public boolean isFinished(){
        return inPose;
    }
}
