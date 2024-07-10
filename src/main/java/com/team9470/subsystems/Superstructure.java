package com.team9470.subsystems;

import com.team9470.shooter.ShotParameters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance;
    private final IntakeRollers intakeRollers = IntakeRollers.getInstance();
    private final IntakeArm intakeArm = IntakeArm.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Swerve swerve = Swerve.getInstance();
    private final Hood hood = Hood.getInstance();
    private final Shooter shooter = Shooter.getInstance();

    public static final boolean SHOOT_ON_MOVE = false;

    public static Superstructure getInstance(){
        if(instance == null){
            instance = new Superstructure();
        }
        return instance;
    }

    @Override
    public void periodic() {
        updateShooter();
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

        hood.setGoal(parameters.angle());
        shooter.setVelocity(parameters.rpm());



    }

    public Command intakeNote(){
        return intakeArm.intakeDown()
                .alongWith(intakeRollers.intakeIn())
                .alongWith(indexer.beltForward()).until(indexer::hasNote)
                .andThen(
                        indexer.beltStop().alongWith(intakeArm.intakeUp()).alongWith(intakeRollers.intakeStop())
                );
    }
}
