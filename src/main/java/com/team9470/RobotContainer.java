// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team9470;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team9470.commands.IntakeNote;
import com.team9470.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
    private final CommandXboxController xboxController = new CommandXboxController(0);

    private final Swerve swerve = new Swerve();
    private final Climber climber = new Climber();
    private final Hood intakeDeploy = new Hood();
    private final IntakeRollers intakeRollers = new IntakeRollers();
    private final Indexer indexer = new Indexer();

    private final SendableChooser<Command> autoChooser;
    public RobotContainer()
    {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData(autoChooser);
    }
    
    
    private void configureBindings() {
        swerve.setDefaultCommand(
                swerve.driveCommand(
                    () -> -xboxController.getLeftX(),
                    () -> -xboxController.getLeftY(),
                    () -> -xboxController.getRightX()
                )
        );

        xboxController.povUp().whileTrue(climber.climberUp());
        xboxController.povDown().whileTrue(climber.climberDown());
        xboxController.leftBumper().whileTrue(new IntakeNote(intakeRollers, intakeDeploy, indexer));
    }
    
    
    public Command getAutonomousCommand()
    {
        return new InstantCommand();
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
