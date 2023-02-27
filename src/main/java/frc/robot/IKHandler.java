package frc.robot;

import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PulleySubsystem;
import frc.robot.util.Constants;

public class IKHandler {
    private LimeLightSubsystem limelight;
    private PulleySubsystem pulley;
    private LiftSubsystem lift;
    private double x, y; // x and y should be desired coordinates, x relative to front edge of robot frame and y relative to floor

    public IKHandler(LimeLightSubsystem limelight, PulleySubsystem pulley, LiftSubsystem lift){
        this.limelight = limelight;
        this.pulley = pulley;
        this.lift = lift;
        x = 0;
        y = 0;
    }

    public boolean setPosition(double desiredX, double desiredY){
        boolean assignment = assignXY(desiredX, desiredY);
        double r = Math.sqrt(Math.pow((x + Constants.LIFT_PIVOT_FROM_FRONT), 2) + Math.pow((y - Constants.LIFT_PIVOT_FROM_FLOOR), 2));
        double theta = Math.atan(y/x);
        lift.setSetPoint(r);
        pulley.setSetPoint(theta);
        return assignment;
    }

    //Returns false if any of the values exceeded a bound, otherwise returns true
    public boolean assignXY(double xval, double yval){
        if (xval > Constants.LIFT_MAX_HORIZONTAL){
            x = Constants.LIFT_MAX_HORIZONTAL;
            return false;
        }
        if (yval > Constants.LIFT_MAX_VERTICAL)
        {
            yval = Constants.LIFT_MAX_VERTICAL;
            return false;
        }
        x = xval;
        y = yval;
        return true;
    }

}
