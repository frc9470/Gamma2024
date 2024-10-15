package com.team9470.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Consts.IntakeConstants.*;

/**
 * Gets notes from the floor into the intake ecosystem
 */
public class IntakeRollers extends SubsystemBase {
    private static IntakeRollers instance;
    public final CANSparkMax intakeRollerMotor = new CANSparkMax(INTAKE_ROLLER_ID, CANSparkMax.MotorType.kBrushless);
    private IntakeRollers(){
        //settings
        intakeRollerMotor.restoreFactoryDefaults();
        intakeRollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        intakeRollerMotor.setInverted(true);
    }

    public static IntakeRollers getInstance(){
        if(instance == null){
            instance = new IntakeRollers();
        }
        return instance;
    }

    public void setVoltage (double voltage){
        intakeRollerMotor.setVoltage(voltage);
    }

    public Command intakeIn(){
        return this.runEnd(() -> setVoltage(INTAKE_TAKE_IN_VOLTAGE.get()), () -> setVoltage(0f));
    }

    public Command intakeStop (){
        return new InstantCommand(() -> setVoltage(0f));
    }

    public Command intakeOut(){
        return this.runEnd(() -> setVoltage(-INTAKE_TAKE_IN_VOLTAGE.get()), () -> setVoltage(0f));
    }
}
