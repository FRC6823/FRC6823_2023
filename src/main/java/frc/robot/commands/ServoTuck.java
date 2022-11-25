package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;

public class ServoTuck extends CommandBase {
    private LimeLightSubsystem limeLightSubsystem;
    private boolean isFinished = false;

    public ServoTuck(LimeLightSubsystem limeLightSubsystem) {
        this.limeLightSubsystem = limeLightSubsystem;
        addRequirements(limeLightSubsystem);
    }

    @Override
    public void execute() {
        limeLightSubsystem.setServoAngle(35);
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        isFinished = false;
    }
}