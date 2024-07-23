package com.team9470.subsystems;

import com.team9470.Consts;
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
    public ShotParameters parameters = null;

    public ShotType defaultType = ShotType.STEADY;
    public ShotType shotType = defaultType;

    public static Superstructure getInstance(){
        if(instance == null){
            instance = new Superstructure();
        }
        return instance;
    }

    @Override
    public void periodic() {
        updateShooter();
        SmartDashboard.putString("FiringParams/ShotType", shotType.name());
    }

    private void updateShooter(){
        Pose2d pose = swerve.getPose();
        ChassisSpeeds speeds = swerve.getRobotVelocity();
        switch(shotType) {
            case AUTO -> {
                try {
                    parameters = ShotParameters.calculate(
                            pose,
                            new Twist2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond),
                            Swerve.isRedAlliance(),
                            SHOOT_ON_MOVE
                    );
                } catch (NullPointerException e) {
                    System.err.println("Error while generating shooter parameters, likely caused by lack of data: " + e.getMessage());
                }

//                if (parameters == null || parameters.distance() > 10.4){
//                    hood.setGoal(1);
//                    shooter.setVelocity(STEADY_RPM);
//                    return;
//                }
            }
            case STEADY -> {
                hood.setGoal(1);
                shooter.setVelocity(STEADY_RPM);
                return;
            }
            case SUBWOOFER -> // for static shots, treat yaw as an adjustment parameter
                    parameters = Consts.ShooterConstants.SUBWOOFER;
            case PODIUM -> parameters = Consts.ShooterConstants.PODIUM;
            case PODIUM_SIDE -> parameters = Consts.ShooterConstants.PODIUM_SIDE;
            case AMP -> {
                // we can't use the static shot parameters method here
                // we must mess with the top and bottom roller tuning to achieve the desired spin effect
                shooter.setVelocity(1000, 2500);
                hood.setGoal(Consts.HoodConstants.AMP_POS);
                return;
            }
            case FEED -> {
                shooter.setVelocity(5400);
                hood.setGoal(1.2);
                return;
            }
            case REVERSE -> {
                shooter.setVelocity(-3000);
                hood.setGoal(0.7);
            }
        }

        hood.setGoal(parameters.angle());
        shooter.setVelocity(parameters.rpm());

        SmartDashboard.putNumber("FiringParams/Distance to target", parameters.distance());
        SmartDashboard.putNumber("FiringParams/HoodError", parameters.angle() - hood.getPosition());
        SmartDashboard.putNumber("FiringParams/TargetHeading", parameters.heading().getDegrees());
        SmartDashboard.putNumber("FiringParams/CurrentHeading", swerve.getHeading().getDegrees());
    }

    public Command intakeNote(){
        return intakeArm.intakeDown().andThen(intakeArm.waitReady())
                .andThen(
                        intakeRollers.intakeIn()
                                .alongWith(indexer.beltForward()).until(indexer::hasNote)
                )
                .andThen(
                        indexer.beltStop().alongWith(intakeArm.intakeUp()).alongWith(intakeRollers.intakeStop())
                )
                .handleInterrupt(() -> {
                    indexer.setVoltage(0);
                    intakeRollers.setVoltage(0);
                    intakeArm.setGoal(Consts.IntakeConstants.UP_GOAL);
                });
    }

    public Command shootNote(){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                    // wait for shooter to spin up
                    shooter.waitReady(),
                    // wait for hood to get to correct angle
                    hood.waitReady()
                    // wait for heading to update
//                    swerve.aimAtYaw(() -> parameters.heading())
                ),
                indexer.beltForward().until(() -> !indexer.hasNote()),
                new WaitCommand(0.3),
                indexer.beltStop()
        );
    }

    public Command staticShot(ShotType type){

        return new SequentialCommandGroup(
                new InstantCommand(() -> shotType = type),
                new ParallelCommandGroup(
                        // wait for shooter to spin up
                        shooter.waitReady(),
                        // wait for hood to get to correct angle
                        hood.waitReady()
                        // wait for heading to update
//                        swerve.aimAtYaw(() -> swerve.getHeading().plus(parameters.heading()))
                ),
                new WaitCommand(1).deadlineWith(indexer.beltMaxForward()),
                new InstantCommand(() -> shotType = defaultType)
        ).handleInterrupt(() -> {shotType = defaultType; indexer.setVoltage(0);});
    }

    public Command autonShot(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> shotType = ShotType.SUBWOOFER),
                new ParallelCommandGroup(
                        // wait for shooter to spin up
                        shooter.waitReady(),
                        // wait for hood to get to correct angle
                        hood.waitReady()
                        // wait for heading to update
                ),
                indexer.beltMaxForward().until(() -> !indexer.hasNote())
        );
    }

    public Command ampShot(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> shotType = ShotType.AMP),
                new ParallelCommandGroup(
                        // wait for shooter to spin up
                        shooter.waitReady(),
                        // wait for hood to get to correct angle
                        hood.waitReady()
                ),
                indexer.beltMaxForward()
        ).handleInterrupt(() -> {shotType = defaultType; indexer.setVoltage(0);});
    }

    public Command feedShot(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> shotType = ShotType.FEED),
                new ParallelCommandGroup(
                        // wait for shooter to spin up
                        shooter.waitReady(),
                        // wait for hood to get to correct angle
                        hood.waitReady(),
                        // wait for heading to update
                        swerve.aimAtFeed()
                ),
                indexer.beltMaxForward()
        ).handleInterrupt(() -> {shotType = defaultType; indexer.setVoltage(0);});
    }

    public Command reverse(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> shotType = ShotType.REVERSE),
                indexer.beltBackward()
        ).handleInterrupt(() -> {shotType = defaultType; indexer.setVoltage(0);});
    }

    public enum ShotType {
        AUTO, SUBWOOFER, STEADY, PODIUM, PODIUM_SIDE, AMP, FEED, REVERSE
    }
}
