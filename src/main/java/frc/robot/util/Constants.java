package frc.robot.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Constants {
    // Default units: Meters (m/s, m/s/s), Degrees 
    // Assume default unless written otherwise
    // "Forward" (on boot) - +X axis
    // "Left" (on boot) - +Y axis

    public static boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);

    //Useful conversion constants
    public static final double degToRad = Math.PI / 180;
    public static final double radToDeg = 180 / Math.PI;


    //Wheel Offsets
    public static final double fLOffset = 220.66015625;
    public static final double fROffset = 205.6640450656414;
    public static final double bLOffset = 23.203123033046722;
    public static final double bROffset = 238.798828125;


    //Swerve wheel module constants (currently using L2 swerve modules)
    public static final double L1_RATIO = 8.14;
    public static final double L2_RATIO = 6.75;
    public static final double L3_RATIO = 6.12;
    public static final double STEER_RATIO = 150.0/7.0;
    public static final double WHEEL_CIRCUMFERENCE = 0.31918581324; 


    //Drivetrain size constants (26' by 32')
    public static final double DRIVE_TRAIN_WIDTH = 0.5334; // left to right size of drivetrain
    public static final double DRIVE_TRAIN_LENGTH = 0.6858; // front to back size of drivetrain


    //Lift/arm max and min values
    public static final double ELEVATOR_MIN = 0;
    public static final double ELEVATOR_MAX = .9;
    public static final double EXTENSION_MIN = -1;
    public static final double EXTENSION_MAX = -105;
    public static final double GRIPPER_MIN = 0.03;
    public static final double GRIPPER_MAX = 0.77;


    //Drivetrain movement maxes
    public static final double kMaxVelocity = 6380 * WHEEL_CIRCUMFERENCE / (60.0 * L2_RATIO);
    public static final double kMaxAccel = kMaxVelocity;
    public static final double kMaxAngularVelocity = kMaxVelocity / Math.hypot(DRIVE_TRAIN_LENGTH / 2.0, DRIVE_TRAIN_WIDTH / 2.0);
    public static final double kMaxAngularAccel = kMaxAngularVelocity;
    public static final TrapezoidProfile.Constraints kTurnControlConstraints = new TrapezoidProfile.Constraints(kMaxAngularVelocity, kMaxAngularAccel);
    public static final SupplyCurrentLimitConfiguration kdriveCurrentLimit = new SupplyCurrentLimitConfiguration(true, 60, 60, 0);
    

    //PID taken from Sonic Squirrels
    //Re-test with full bot
    public static final double kP = 2.2941;//2.2941
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kA = 0.435;
    public static final double kV = 2.344;
    public static final double kS = 0.628;

    public static final double kPThetaController = 2.0; //1.5;
    public static final double kIThetaController = 0.0;
    public static final double kDThetaController = 0.0;


    //Yaw pid constants (no kd)
    public static final double yawKp = 0.17;
    public static final double yawKi = 0.01;


    //Apriltag relative PID setpoint constants for LineUp
    //[0] - TZ, [1] - TX, [2] - yaw
    public static final double[] leftScore = new double[]{-0.88, 0.54, 0}; //180
    public static final double[] rightScore = new double[]{-0.88, -0.58, 0};
    public static final double[] leftPickup = new double[]{-0.935, 0.71, 0}; 
    public static final double[] rightPickup = new double[]{-0.935, -0.71, 0};

    //AprilTag max TX value
    public static final double TX_MAX = .83;


    //Lift/arm set poses to be used by PositionHandler
    public static final double[] highScorePose = new double[] {0.808605010509491,-104.5,0.4887945950031288};
    //public static final double[] lowScorePose = new double[] {0.8156105103969574,-37.76153564453125,0.488794595003128};
    public static final double[] lowScorePose = new double[] {0.809765040874481,-30.428321838378906,0.495485693216324};
    public static final double[] pickupPose = new double[] {0.81150432062149,-20.809432983398438,0.491733729839325};
    public static final double[] transportPose = new double[]{0.88200432062149, -10, 0.552061796188354};
    // public static final double[] floorPose = new double[] {0.684879839420319,-12.880990028381348,0.404875934123993};  //old floor pose -Bruce
    public static final double[] floorPose = new double[] {0.684879839420319,-12.880990028381348,0.35}; //new floor pose -Bruce
    //public static final double[] startPose = new double[] {0.900431036949158,-20,0.120892271399498}; //old position - BEn
    public static final double[] startPose = new double[] {0.888,-20,0.123850924730301}; //adjusted position - Ben


    //Path planning node poses in field space
    public static final double[] node1Pose = new double[] {0,0};
    public static final double[] node2Pose = new double[] {0,0};
    public static final double[] node3Pose = new double[] {0,0};
    public static final double[] node4Pose = new double[] {0,0};
    public static final double[] node5Pose = new double[] {0,0};
    public static final double[] node6Pose = new double[] {0,0};
    public static final double[] node7Pose = new double[] {0,0};
    public static final double[] node8Pose = new double[] {0,0};
    public static final double[] node9Pose = new double[] {0,0};
}