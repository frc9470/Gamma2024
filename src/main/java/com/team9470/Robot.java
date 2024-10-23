// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team9470;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.urcl.URCL;

public class Robot extends TimedRobot
{
    private Command currAuton;
    
    private RobotContainer robotContainer;
    boolean enabled = false;
    
    
    @Override
    public void robotInit() {
        URCL.start();
        robotContainer = new RobotContainer();
        LogUtil.periodicWarning(() -> !enabled && robotContainer.getAutonomousCommand().getName().equals("None auto command"), "[WARNING] NO AUTONOMOUS COMMAND SET. ROBOT WILL NOT MOVE AUTONOMOUSLY. \n[WARNING]SET AN AUTONOMOUS COMMAND VIA SHUFFLEBOARD -> SmartDashboard/AutoChooser");
    }

    int a = 0;
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
//        LogUtil.periodic();
    }
    
    
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {}
    
    
    @Override
    public void disabledExit() {
        enabled = true;
    }
    
    
    @Override
    public void autonomousInit()
    {
        currAuton = robotContainer.getAutonomousCommand();

        // basically if the auton selector has not selected anything,
        // this won't run!
        if (currAuton != null)
        {
            currAuton.schedule();
        }
    }
    
    
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void autonomousExit() {}
    
    
    @Override
    public void teleopInit()
    {
        if (currAuton != null)
        {
            currAuton.cancel();
        }
    }
    
    
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void teleopExit() {}
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void testExit() {}
}
