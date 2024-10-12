// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team9470;

import com.pathplanner.lib.auto.NamedCommands;
import com.team9470.subsystems.*;
import com.team9470.subsystems.vision.Vision;
import edu.wpi.first.math.MathUtil;
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
    private final IntakeRollers intakeRollers = IntakeRollers.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Vision vision = Vision.getInstance();

    private final Superstructure superstructure = new Superstructure();

    private final SendableChooser<Command> autoChooser;
    public RobotContainer()
    {
        initPathplanner();
        configureBindings();
        autoChooser = Util.buildAutoChooser("", (String name) -> name.contains("N"));
        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    private void initPathplanner() {
        NamedCommands.registerCommand("intake", superstructure.intakeNote());
        NamedCommands.registerCommand("shoot", superstructure.autonShot());
    }

    private void configureBindings() {
        swerve.setDefaultCommand(
                swerve.driveCommand(
                        () -> -MathUtil.applyDeadband(xboxController.getLeftY(),
                                Consts.SwerveConstants.DEADBAND),
                        () -> -MathUtil.applyDeadband(xboxController.getLeftX(),
                                Consts.SwerveConstants.DEADBAND),
                        () -> -MathUtil.applyDeadband(xboxController.getRightX(),
                                Consts.SwerveConstants.DEADBAND)
                )
        );

        xboxController.leftBumper().whileTrue(superstructure.intakeNote());
        xboxController.rightBumper().whileTrue(superstructure.intakeAmp());
        xboxController.rightTrigger().whileTrue(superstructure.staticShot(Superstructure.ShotType.SUBWOOFER));
        xboxController.leftTrigger().whileTrue(superstructure.ampNote());

        xboxController.povUp().whileTrue(superstructure.staticShot(Superstructure.ShotType.PODIUM));

        xboxController.a().onTrue(new InstantCommand(swerve::zeroGyroWithAlliance));

        xboxController.x().whileTrue(superstructure.shootNote());
        xboxController.y().whileTrue(indexer.beltForward());
        xboxController.b().whileTrue(superstructure.climb());


//        xboxController.x().whileTrue(
//                        shooter.getQuasistatic(SysIdRoutine.Direction.kForward)
//        );
//        xboxController.y().whileTrue(
//                shooter.getQuasistatic(SysIdRoutine.Direction.kReverse)
//        );
//        xboxController.b().whileTrue(
//                shooter.getDynamic(SysIdRoutine.Direction.kForward)
//        );
//        xboxController.a().whileTrue(
//                        shooter.getDynamic(SysIdRoutine.Direction.kReverse)
//        );
//        xboxController.povUp().whileTrue(hood.angleCommand(1));
//        xboxController.povDown().whileTrue(hood.angleCommand(.4));
//        xboxController.povRight().whileTrue(intakeRollers.intakeIn());
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
        return autoChooser.getSelected();
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
