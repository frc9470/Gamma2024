package com.team9470.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    private final IntakeRollers intakeRollers;
    private final IntakeArm intakeArm;
    private final Indexer indexer;

    public Superstructure(Swerve swerve, Hood hood, Shooter shooter, IntakeRollers intakeRollers, IntakeArm intakeArm, Indexer indexer) {
        this.intakeRollers = intakeRollers;
        this.intakeArm = intakeArm;
        this.indexer = indexer;
    }

    @Override
    public void periodic() {
        updateShooter();
    }

    private void updateShooter(){
        Pose
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
