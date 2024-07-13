package com.team9470.subsystems;

import com.team9470.Constants;
import com.team9470.shooter.ShotParameters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance;
    private final IntakeRollers intakeRollers = IntakeRollers.getInstance();
    private final IntakeArm intakeArm = IntakeArm.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Swerve swerve = Swerve.getInstance();
    private final Hood hood = Hood.getInstance();
    private final Shooter shooter = Shooter.getInstance();

    public static final boolean SHOOT_ON_MOVE = false;
    public static final double STEADY_RPM = 120.0;

    public static Superstructure getInstance(){
        if(instance == null){
            instance = new Superstructure();
        }
        return instance;
    }

    @Override
    public void periodic() {
//        updateShooter();
    }

    private void updateShooter(){
        Pose2d pose = swerve.getPose();
        ChassisSpeeds speeds = swerve.getRobotVelocity();
        ShotParameters parameters = ShotParameters.calculate(
                pose,
                new Twist2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond),
                Swerve.isRedAlliance(),
                SHOOT_ON_MOVE
        );

        if (parameters.distance() > 10.4){
            hood.setGoal(50.0);
            shooter.setVelocity(STEADY_RPM);
            return;
        }

        hood.setGoal(parameters.angle());
        shooter.setVelocity(parameters.rpm());

        SmartDashboard.putNumber("FiringParams/Distance to target", parameters.distance());
        SmartDashboard.putNumber("FiringParams/HoodError", parameters.angle() - hood.getPosition());
        SmartDashboard.putNumber("FiringParams/TargetHeading", parameters.heading().getDegrees());
        SmartDashboard.putNumber("FiringParams/CurrentHeading", swerve.getHeading().getDegrees());
    }

    public Command intakeNote(){
        return intakeArm.intakeDown()
                .alongWith(intakeRollers.intakeIn())
                .alongWith(indexer.beltForward()).until(indexer::hasNote)
                .andThen(
                        indexer.beltStop().alongWith(intakeArm.intakeUp()).alongWith(intakeRollers.intakeStop())
                ).handleInterrupt(() -> {
                    indexer.setVoltage(0);
                    intakeRollers.setVoltage(0);
                    intakeArm.setGoal(Constants.IntakeConstants.UP_GOAL);
                });
    }

    public Command shootNote(){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                    // wait for shooter to spin up
                    shooter.waitReady(),
                    // wait for hood to get to correct angle
                    hood.waitReady(),
                    // wait for heading to update
                    swerve.aimAtSpeaker()
                ),
                indexer.beltForward().until(() -> !indexer.hasNote()),
                new WaitCommand(0.2),
                indexer.beltStop()
        );
    }
}
