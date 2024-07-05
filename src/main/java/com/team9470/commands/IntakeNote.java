package com.team9470.commands;


import com.team9470.subsystems.Indexer;
import com.team9470.subsystems.Hood;
import com.team9470.subsystems.IntakeRollers;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeNote extends SequentialCommandGroup {
    public IntakeNote (IntakeRollers intakeRollers, Hood intakeDeploy, Indexer indexer){
        addCommands(
                intakeDeploy.intakeDown()
                        .alongWith(intakeRollers.intakeIn())
                        .alongWith(indexer.beltForward()).until(indexer::hasNote),
                indexer.beltStop().alongWith(intakeDeploy.intakeUp()).alongWith(intakeRollers.intakeStop())
        );
    }
}
