package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Load extends CommandBase {
    private ShooterSubsystem shooter;
    private double loadPower;
    private ConveyorSubsystem conveyor;

    public Load(ShooterSubsystem shooter, ConveyorSubsystem conveyor) {
        //Instantiate subsystem, Joystick Handler
        this.shooter = shooter;
        this.conveyor = conveyor;

        loadPower = 0.3;
    }

    @Override
    public void execute() {
        shooter.backLoad(loadPower);
        conveyor.backConvey();
    }

    public void stop(){
        shooter.loadStop();
        conveyor.stopConvey();
    }
}
