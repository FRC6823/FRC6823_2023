package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class PickUpBall extends CommandBase {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private LimeLightSubsystem limeLightSubsystem;

    private PIDController distController, aimController;

    private long whenStartedIntaking;
    private int stage;
    private int pipeline;
    private boolean isItFinished;

    public PickUpBall(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem,
            LimeLightSubsystem limeLightSubsystem, int pipeline) {

        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.limeLightSubsystem = limeLightSubsystem;

        stage = 0;
        this.pipeline = pipeline;
        isItFinished = false;

        addRequirements(swerveDriveSubsystem, intakeSubsystem, limeLightSubsystem);
    }

    public int getStage() {
        return stage;
    }

    private boolean hasSeenBall = false;

    @Override
    public void execute() {
        stage = stage == -1 ? 0 : stage;

        double distanceCommand = distController.calculate(limeLightSubsystem.getTy());
        double aimCommand = aimController.calculate(limeLightSubsystem.getTx());

        limeLightSubsystem.setPipeline(pipeline);
        //SmartDashboard.putNumber("Ball Eat Stage", stage);
        //SmartDashboard.putNumber("Ball Distance", distanceCommand);
        //SmartDashboard.putNumber("Aim Command Ball", aimCommand);

        if (limeLightSubsystem.hasTarget()) {
            hasSeenBall = true;
        }
        if (stage == 0) {
            // far from ball, need to move towards it using limelight
            swerveDriveSubsystem.drive(0, distanceCommand, aimCommand * -1);

            if (Math.abs(distController.getPositionError()) < 5 && hasSeenBall) {
                stage = 1;
                whenStartedIntaking = System.currentTimeMillis();
                intakeSubsystem.intake();
            }
            if (!hasSeenBall) {
                isItFinished = true;
            }
        } else if (stage == 1) {
            // close to ball, move towards it despite not seeing it
            swerveDriveSubsystem.drive(0, -0.1, 0);

            // stop after 2 seconds
            if (System.currentTimeMillis() - whenStartedIntaking >= 1000) {
                isItFinished = true;
                intakeSubsystem.stopIntake();
                swerveDriveSubsystem.stop();

            }
        }
    }

    @Override
    public void initialize() {
        limeLightSubsystem.setPipeline(1);
        aimController = new PIDController(.016, 0, 0);
        distController = new PIDController(.05, 0, 0);

        distController.setSetpoint(0);
        aimController.setSetpoint(0);

        stage = 0;
    }

    @Override
    public boolean isFinished() {
        return isItFinished;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
        intakeSubsystem.stopIntake();
        stage = 0;
        isItFinished = false;
        hasSeenBall = false;
    }
}