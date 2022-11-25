package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;

public class SwitchPipelineCommand extends CommandBase {
    //Declare subsystem, isFinished, and intended pipeline
    private LimeLightSubsystem limeLightSubsystem;
    private boolean isFinished = false;
    private int pipeline;

    public SwitchPipelineCommand(LimeLightSubsystem limeLightSubsystem, int pipeline) {
        //Instantiate subsystem, isFinished, and intended pipeline
        this.limeLightSubsystem = limeLightSubsystem;
        this.pipeline = pipeline;
        limeLightSubsystem.setPipeline(pipeline);
    }

    @Override
    public void execute() {
        limeLightSubsystem.setPipeline(pipeline);
        //Adjusts servo to angle appropriate for selected pipeline
        // if (pipeline == 0)
        //     limeLightSubsystem.setServoAngle(65);
        // else if (pipeline == 1)
        //     limeLightSubsystem.setServoAngle(15);
        // if (pipeline == 1 && Math.abs(limeLightSubsystem.getServoAngle() - 15) < 1)
        //     isFinished = true;
        // else if (pipeline == 0 && Math.abs(limeLightSubsystem.getServoAngle() - 65) < 1)
        //     isFinished = true;
        //     SmartDashboard.putNumber("The Servo Angle", limeLightSubsystem.getPipeline());

    }

    @Override
    public void initialize() {
        limeLightSubsystem.setPipeline(pipeline);
        // if (pipeline == 0)
        //     limeLightSubsystem.setServoAngle(65);
        // else if (pipeline == 1)
        //     limeLightSubsystem.setServoAngle(15);
        //SmartDashboard.putNumber("The Servo Angle", limeLightSubsystem.getPipeline());

    }

    @Override
    public boolean isFinished() {
        //SmartDashboard.putNumber("The Servo Angle", limeLightSubsystem.getPipeline());
        return isFinished;
    }

    @Override
    public void end(boolean inturrupted) {
        isFinished = false;
    }
}