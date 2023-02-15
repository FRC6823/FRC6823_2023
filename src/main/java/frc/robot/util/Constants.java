package frc.robot.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class Constants {
    public static final double degToRad = Math.PI / 180;
    public static final double radToDeg = 180 / Math.PI;
    public static final double fLOffset = 305.7714584544301;
    public static final double fROffset = 120.32225542515516;
    public static final double bLOffset = 337.0605183020234;
    public static final double bROffset = 27.509763292968273;

    public static final double L1_RATIO = 8.14;
    public static final double L2_RATIO = 6.75;
    public static final double L3_RATIO = 6.12;
    public static final double STEER_RATIO = 150.0/7.0;
    public static final double WHEEL_CIRCUMFERENCE = 0.31918581324; // Wheel circumference in meters, measure this later

    //Drive Constants to be used in path planning (WIP)
    public static final double kMaxSpeed = .1;
    public static final double kMaxAccel = 0.05;
    public static final TrapezoidProfile.Constraints kTurnControlConstraints = new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAccel);
}
