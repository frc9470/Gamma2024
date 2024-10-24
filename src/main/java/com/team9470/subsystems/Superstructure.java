package com.team9470.subsystems;

import com.team9470.Consts;
import com.team9470.shooter.ShotParameters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

/**
 * OVERALL intake superstructure (for OVERALL control)
 * <br><br>
 * <b>IMPORTANT NOTES:</b><br>
 * - SHOOTER IS ALWAYS SPINNING; THIS WILL OCCUR THE MOMENT THE ROBOT IS ENABLED
 */
public class Superstructure extends SubsystemBase {
    // initializes singleton of the class
    private static Superstructure instance;


    // SUBSYSTEMS

    private final IntakeRollers intakeRollers = IntakeRollers.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Swerve swerve = Swerve.getInstance();
    private final Hood hood = Hood.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Ampevator ampevator = Ampevator.getInstance();

    // TODO: NOT USING FOR NOW

    private final Climber climber = Climber.getInstance();


    // ADDITIONAL CONTROLLERS

    public ShotParameters parameters = null;

    // CONSTANTS

    /**
     * it will compensate for robot's movement WHILE shooting
     * <br>
     * <b>TODO:</b> based off of citrus circuits' code; see if it works!
     */
    public static final boolean SHOOT_ON_MOVE = false;
    /**
     * the rpm that the shooter will spin at when
     * the shooter <i>isn't being used for anything</i>
     */
    public static final double STEADY_RPM = 120.0;


    // RUNTIME VARIABLES

    public ShotType defaultType = ShotType.STEADY;
    public ShotType shotType = defaultType;

    // RUNTIME VARIABLES

    private int count = 0;

    public static Superstructure getInstance(){
        if (instance == null) {
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

                if (parameters == null || parameters.distance() > 10.4){
                    hood.setGoalDegrees(45);
                    shooter.setVelocity(STEADY_RPM);
                    return;
                }

                if(count%20 == 0){
                    System.out.println(parameters);
                }
                count++;
            }
            case STEADY -> {
                hood.setGoalDegrees(30);
                shooter.setVelocity(STEADY_RPM);
                return;
            }
            case SUBWOOFER -> // for static shots, treat yaw as an adjustment parameter
                    parameters = Consts.ShooterConstants.SUBWOOFER;
            case PODIUM -> parameters = Consts.ShooterConstants.PODIUM;
            case PODIUM_SIDE -> parameters = Consts.ShooterConstants.PODIUM_SIDE;
            case FEED -> { // update to use actual regression maps
                shooter.setVelocity(5400);
                hood.setGoalDegrees(30);
            }
            case REVERSE -> { // broken
                shooter.setVelocity(-3000);
                hood.setGoalDegrees(40);

            }
            case CLIMBING -> {
                shooter.setVelocity(0);
                hood.setGoalDegrees(45);
            }
        }

        hood.setGoalDegrees(parameters.angle());
//        shooter.setVelocity(parameters.rpm());

        SmartDashboard.putNumber("FiringParams/Distance to target", parameters.distance());
        SmartDashboard.putNumber("FiringParams/HoodError", parameters.angle() - hood.getPositionDegrees());
        SmartDashboard.putNumber("FiringParams/TargetHeading", parameters.heading().getDegrees());
        SmartDashboard.putNumber("FiringParams/CurrentHeading", swerve.getHeading().getDegrees());
    }

    public Command intakeNote(){
        return intakeRollers.intakeIn()
                    .alongWith(indexer.beltThrough()).until(indexer::hasNote)
//                .andThen(indexer.beltThrough().alongWith(intakeRollers.intakeIn()).withTimeout(.3))
                .andThen(
                        indexer.beltStop().alongWith(intakeRollers.intakeStop())
                )
                .handleInterrupt(() -> {
                    indexer.setBottom(0);
                    intakeRollers.setVoltage(0);
                });
    }

    public Command shootNote(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> shotType = ShotType.AUTO),
                new InstantCommand(() -> System.out.println("Shooting, shotparameters " + parameters)),
                new ParallelCommandGroup(
                    swerve.aimAtYaw(() -> parameters != null ? parameters.heading() : new Rotation2d()).onlyIf(() -> parameters != null),
                    // wait for shooter to spin up
//                    shooter.waitReady(),
                    // wait for hood to get to correct angle
                    hood.waitReady()

                ),
                indexer.beltMaxForward().until(() -> !indexer.hasNote()),
                new WaitCommand(0.3),
                indexer.beltStop(),
                new InstantCommand(() -> shotType = defaultType)
        ).handleInterrupt(() -> shotType = defaultType);
    }

    public Command staticShot(ShotType type){

        return new SequentialCommandGroup(
                new InstantCommand(() -> shotType = type),
                new ParallelCommandGroup(
                        // wait for shooter to spin up
//                        shooter.waitReady()
                        // wait for hood to get to correct angle
                        hood.waitReady()
                        // wait for heading to update
//                        swerve.aimAtYaw(() -> swerve.getHeading().plus(parameters.heading()))
                ),
                new WaitCommand(1).deadlineWith(indexer.beltMaxForward().alongWith(intakeRollers.intakeIn())),
                new InstantCommand(() -> shotType = defaultType)
        ).handleInterrupt(() -> {shotType = defaultType; indexer.setBottom(0); intakeRollers.intakeStop();});
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
        ).handleInterrupt(() -> {shotType = defaultType; indexer.setBottom(0);});
    }

    public Command reverse(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> shotType = ShotType.REVERSE),
                indexer.beltBackward().alongWith(intakeRollers.intakeOut()).alongWith(ampevator.rollerIn())
        ).handleInterrupt(() -> {shotType = defaultType; indexer.setBottom(0); intakeRollers.intakeStop(); ampevator.rollerStop();});
    }

    public Command intakeAmp(){
        return indexer.beltForward()
                .alongWith(ampevator.rollerOut())
                .alongWith(intakeRollers.intakeIn())
                .until(ampevator::hasNote)
                .andThen(new WaitCommand(0.2) // Wait for 0.5 seconds
                        .deadlineWith(
                                intakeRollers.intakeIn(), // Continue running the rollers during the wait
                                ampevator.rollerOut()
                        )
                );
    }

    public Command ampNote(){
        return ampevator.ampNote().handleInterrupt(() -> ampevator.setGoal(0));
    }

    /**
     * command that has the following sequence
     * - pivot shooter all the way up
     * - set shooter to 0 rpm
     * - climber up
     * - ampevator up
     * @return
     */
    public Command prepareClimb() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shotType = ShotType.CLIMBING),
                new InstantCommand(() -> climbState = ClimbState.CLIMB),
                ampevator.setTarget(Consts.AmpevatorConstants.EXTENSION_HEIGHT),
                climber.prepareClimb()
        );
    }

    public Command doClimb(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> climbState = ClimbState.TRAP),
                climber.climb()
        );
    }

    public enum ClimbState {
        PREPARE, CLIMB, TRAP
    }
    private ClimbState climbState = ClimbState.PREPARE;

    public Command climb(){
        return new ConditionalCommand(
                prepareClimb(),
                new ConditionalCommand(
                        doClimb(),
                        ampevator.rollerOut(),
                        () -> climbState == ClimbState.CLIMB
                ),
                () -> climbState == ClimbState.PREPARE
        );
    }


    public enum ShotType {
        AUTO, SUBWOOFER, STEADY, PODIUM, PODIUM_SIDE, AMP, FEED, REVERSE, CLIMBING
    }
}
