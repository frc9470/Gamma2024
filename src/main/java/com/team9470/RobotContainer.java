// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team9470;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team9470.subsystems.*;
import com.team9470.subsystems.vision.Vision;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
    private final CommandXboxController xboxController = new CommandXboxController(0);

    private final Swerve swerve = Swerve.getInstance();
    private final Hood hood = Hood.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final IntakeArm intakeArm = IntakeArm.getInstance();
    private final IntakeRollers intakeRollers = IntakeRollers.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Vision vision = Vision.getInstance();

    private final Superstructure superstructure = new Superstructure();

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
                        xboxController::getLeftY,
                    () -> -xboxController.getLeftX(),
                    () -> -xboxController.getRightX()
                )
        );

        xboxController.a().onTrue(new InstantCommand(swerve::zeroGyro));

        xboxController.leftBumper().whileTrue(superstructure.intakeNote());
        xboxController.rightTrigger().whileTrue(superstructure.shootNote());
//        xboxController.povUp().whileTrue(intakeArm.intakeUp());
//        xboxController.povDown().whileTrue(intakeArm.intakeDown());
//        xboxController.povRight().whileTrue(intakeRollers.intakeIn());
//
//        xboxController.x().whileTrue(
//                intakeArm.getQuasistatic(SysIdRoutine.Direction.kForward).andThen(new WaitCommand(1))
//                        .andThen(intakeArm.getQuasistatic(SysIdRoutine.Direction.kReverse)).andThen(new WaitCommand(1))
//                        .andThen(intakeArm.getDynamic(SysIdRoutine.Direction.kForward)).andThen(new WaitCommand(1))
//                        .andThen(intakeArm.getDynamic(SysIdRoutine.Direction.kReverse))
//        );
//
//        xboxController.y().whileTrue(
//                swerve.sysIdAngleMotorCommand()
//        );
//
//        xboxController.b().whileTrue(
//                swerve.sysIdDriveMotorCommand()
//        );
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
