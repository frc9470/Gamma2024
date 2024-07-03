// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team9470;

import com.team9470.commands.IntakeNote;
import com.team9470.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
    private final CommandXboxController xboxController = new CommandXboxController(0);

    private final Swerve swerve = new Swerve();
    private final Climber climber = new Climber();
    private final IntakeDeploy intakeDeploy = new IntakeDeploy();
    private final IntakeRollers intakeRollers = new IntakeRollers();
    private final Indexer indexer = new Indexer();
    public RobotContainer()
    {
        configureBindings();
    }
    
    
    private void configureBindings() {
        swerve.setDefaultCommand(swerve.driveCommand(xboxController::getLeftX, xboxController::getLeftY, xboxController::getRightX));

        xboxController.povUp().whileTrue(climber.climberUp());
        xboxController.povDown().whileTrue(climber.climberDown());
        xboxController.leftBumper().whileTrue(new IntakeNote(intakeRollers, intakeDeploy, indexer));
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}

/*
 * good autos
 * full field localization
 * control theory - how to move arms
 * shooter regression
 *
 * basic robot functions with states
 *  - swerve
 *  - intake
 *  - shooter
 */
