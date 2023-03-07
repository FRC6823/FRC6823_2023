/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Constants;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        // rgb1 = new RGB(9);
        // rgb2 = new RGB(9);
        // rgb1.setPattern(0.67);
        // rgb2.setPattern(0.87);

        // PREFS.putBoolean("DEBUG_MODE", false);
        //SmartDashboard.putBoolean("LemonPipeline", false);

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        robotContainer.getAutoCommandGroup().schedule();
        // robotContainer.getMoreCommands().schedule();

    }

    @Override
    public void teleopInit() {
        robotContainer.getAutoCommandGroup().cancel();
    }

}
