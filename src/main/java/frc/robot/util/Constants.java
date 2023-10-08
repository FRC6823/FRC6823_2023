package frc.robot.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    // Default units: Meters (m/s, m/s/s), Degrees 
    // Assume default unless written otherwise
    // "Forward" (on boot) - +X axis
    // "Left" (on boot) - +Y axis

    public static final double degToRad = Math.PI / 180;
    public static final double radToDeg = 180 / Math.PI;
    public static final double fLOffset = 172.6171875;
    public static final double fROffset = 89.560546875;
    public static final double bLOffset = 299.8828125;
    public static final double bROffset = 165.234375;

    public static final double L1_RATIO = 8.14;
    public static final double L2_RATIO = 6.75;
    public static final double L3_RATIO = 6.12;
    public static final double STEER_RATIO = 150.0/7.0;
    public static final double WHEEL_CIRCUMFERENCE = 0.31918581324; 

    public static final double DRIVE_TRAIN_WIDTH = 0.5334; // left to right size of drivetrain
    public static final double DRIVE_TRAIN_LENGTH = 0.6858; // front to back size of drivetrain

    public static final double kMaxVelocity = 5;
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


