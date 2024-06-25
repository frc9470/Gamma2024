package com.team9470.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Constants.IndexerConstants.*;

public class Indexer extends SubsystemBase {
    public final CANSparkMax index1 = new CANSparkMax(INDEX_ID_1, CANSparkMax.MotorType.kBrushless);
    public final CANSparkMax index2 = new CANSparkMax(INDEX_ID_2, CANSparkMax.MotorType.kBrushless);
    public final DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_ID);
    public Indexer(){
        //settings
        index1.restoreFactoryDefaults();
        index1.setIdleMode(CANSparkBase.IdleMode.kBrake);
        index2.restoreFactoryDefaults();
        index2.setIdleMode(CANSparkBase.IdleMode.kBrake);
        index2.setInverted(true);
    }
    //functions and commands
    public void setVoltage (Float voltage){
        index1.setVoltage(voltage);
        index2.setVoltage(voltage);
    }
    public Command beltForward (){
        return this.runEnd(() -> setVoltage(BELT_FORWARD_VOLTAGE), () -> setVoltage(0f));
    }
    public Command beltBackward (){
        return this.runEnd(() -> setVoltage(BELT_BACKWARD_VOLTAGE), () -> setVoltage(0f));
    }
    public Command beltStop (){
        return new InstantCommand(() -> setVoltage(0f));
    }
    public boolean hasNote(){
        return beamBreak.get();
    }
}
