package frc.robot.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    // Default units: Meters (m/s, m/s/s), Degrees 
    // Assume default unless written otherwise
    // "Forward" (on boot) - +X axis
    // "Left" (on boot) - +Y axis

    public static final double degToRad = Math.PI / 180;
    public static final double radToDeg = 180 / Math.PI;
    public static final double fLOffset = 174.0234375;
    public static final double fROffset = 92.197265625;
    public static final double bLOffset = 302.6953125;
    public static final double bROffset = 170.68359375;

    public static final double L1_RATIO = 8.14;
    public static final double L2_RATIO = 6.75;
    public static final double L3_RATIO = 6.12;
    public static final double STEER_RATIO = 150.0/7.0;
    public static final double WHEEL_CIRCUMFERENCE = 0.31918581324; 

    public static final double DRIVE_TRAIN_WIDTH = .737; // left to right size of drivetrain
    public static final double DRIVE_TRAIN_LENGTH = .737; // front to back size of drivetrain

    public static final double kMaxVelocity = 6380 * WHEEL_CIRCUMFERENCE / (60.0 * L2_RATIO);
    public static final double kMaxAccel = kMaxVelocity;
    public static final double kMaxAngularVelocity = kMaxVelocity / Math.hypot(DRIVE_TRAIN_LENGTH / 2.0, DRIVE_TRAIN_WIDTH / 2.0);
    public static final double kMaxAngularAccel = kMaxAngularVelocity;
    public static final TrapezoidProfile.Constraints kTurnControlConstraints = new TrapezoidProfile.Constraints(kMaxAngularVelocity, kMaxAngularAccel);
    
    //PID taken from Sonic Squirrels
    //Re-test with full bot
    public static final double kP = 2.2941;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kA = 0.435;
    public static final double kV = 2.344;
    public static final double kS = 0.628;

    public static final double kPThetaController = 2.0; //1.5;
    public static final double kIThetaController = 0.0;
    public static final double kDThetaController = 0.0;
}
