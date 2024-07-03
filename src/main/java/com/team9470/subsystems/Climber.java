package com.team9470.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Constants.ClimberConstants.*;

/**
 * Climber Arm Up = POV up
 * Climber Arm Down = POV down
 * Motor IDs = 21 and 22
 */

public class                                                                                                                                                                   Climber extends SubsystemBase {
    public final CANSparkMax winch1 = new CANSparkMax(WINCH_ID_1, CANSparkMax.MotorType.kBrushless);
    public final CANSparkMax winch2 = new CANSparkMax(WINCH_ID_2, CANSparkMax.MotorType.kBrushless);
    public Climber(){
        //settings
        winch1.restoreFactoryDefaults();
        winch1.setIdleMode(CANSparkBase.IdleMode.kBrake);
        winch2.restoreFactoryDefaults();
        winch2.setIdleMode(CANSparkBase.IdleMode.kBrake);
        winch2.setInverted(true);
    }
    //functions and commands
    public void setVoltage (Float voltage){
        winch1.setVoltage(voltage);
        winch2.setVoltage(voltage);
    }
    public Command climberUp (){
        return this.runEnd(() -> setVoltage(CLIMBER_UP_VOLTAGE), () -> setVoltage(0f));
    }
    public Command climberDown (){
        return this.runEnd(() -> setVoltage(CLIMBER_DOWN_VOLTAGE), () -> setVoltage(0f));
    }
}
