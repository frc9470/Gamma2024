package com.team9470.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Constants.IntakeConstants.*;

public class IntakeRollers extends SubsystemBase {
    public final CANSparkMax intakeRollerMotor = new CANSparkMax(INTAKE_ROLLER_ID, CANSparkMax.MotorType.kBrushless);
    public IntakeRollers(){
        //settings
        intakeRollerMotor.restoreFactoryDefaults();
        intakeRollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }
    public void setVoltage (double voltage){
        intakeRollerMotor.setVoltage(voltage);
    }

    public Command intakeIn(){
        return this.runEnd(() -> setVoltage(INTAKE_TAKE_IN_VOLTAGE), () -> setVoltage(0f));
    }

    public Command intakeStop (){
        return new InstantCommand(() -> setVoltage(0f));
    }
}
