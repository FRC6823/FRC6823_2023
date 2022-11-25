package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoystickHandler;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {
    private ShooterSubsystem shooter;
    private ConveyorSubsystem conveyor;
    private JoystickHandler joystickHandler;
    private double shooterAngle;

    public Shoot(ShooterSubsystem shooter, ConveyorSubsystem conveyor,
    JoystickHandler joystickHandler) {
        //Instantiate subsystem, Joystick Handler
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.joystickHandler = joystickHandler;
        shooterAngle = 60;

        addRequirements(this.shooter);
    }

    @Override
    public void execute() {
        joystickHandler.updateDeadZone();
        double loadRate;
        int shootRateLeft;
        int shootRateRight;
        if(joystickHandler.getAxis3() != 0){
            loadRate = shooter.getLoadPercent();
            shootRateLeft = shooter.getShooterRPMLeft()*20;
            shootRateRight = shooter.getShooterRPMRight()*20;
            // shootRateLeft = (int)160000;
            // shootRateRight = (int)160000;
            //shootRate = 1.0;
            conveyor.convey();
        }else if (joystickHandler.getAxis2() != 0){
            loadRate = 0;
            shootRateLeft = 0;
            shootRateRight = 0;
            conveyor.convey();
        }else{
            loadRate = 0;
            shootRateLeft = 0;
            shootRateRight = 0;
            conveyor.stopConvey();
        }
      
        if (joystickHandler.getAxis1() < -0.75){
            shooterAngle = 70;
        }else if (joystickHandler.getAxis1() > 0.75){
            shooterAngle = 50;
        // }else if (joystickHandler.getAxis0() < -0.75){
        //     shooterAngle = 65;
        }else if (joystickHandler.getAxis0() > 0.75){
            shooterAngle = 60;
        }
        shooter.setShooterAngle(shooterAngle);
        shooter.prep(loadRate);
        shooter.shoot(shootRateLeft, shootRateRight);
    }
}