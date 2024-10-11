package com.team9470.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Consts.IndexerConstants.*;

public class Indexer extends SubsystemBase {
    private static Indexer instance;
    public final CANSparkMax bottomRollers = new CANSparkMax(BOTTOM_ROLLER_ID, CANSparkMax.MotorType.kBrushless);
    public final CANSparkMax topRollers = new CANSparkMax(TOP_ROLLER_ID, CANSparkMax.MotorType.kBrushless);
    public final DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_ID);
    private Indexer(){
        //settings
        bottomRollers.restoreFactoryDefaults();
        bottomRollers.setIdleMode(CANSparkBase.IdleMode.kBrake);

        topRollers.restoreFactoryDefaults();
        topRollers.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }


    public static Indexer getInstance(){
        if(instance == null){
            instance = new Indexer();
        }
        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Indexer/Beam Break", hasNote());
    }

    //functions and commands
    public void setTop(double voltage){
        topRollers.setVoltage(voltage);
    }
    public void setBottom(double voltage){
        bottomRollers.setVoltage(voltage);
    }

    public Command beltForward (){ // up to amp
        return this.runEnd(() -> { setTop(-FORWARD_VOLTAGE); setBottom(FORWARD_VOLTAGE);}, () -> setBottom(0.0));
    }

    public Command beltThrough (){ // through
        return this.runEnd(() -> { setTop(FORWARD_VOLTAGE); setBottom(FORWARD_VOLTAGE);}, () -> setBottom(0.0));
    }

    public Command beltMaxForward (){
        return this.runEnd(() -> { setTop(BELT_MAX_FORWARD_VOLTAGE); setBottom(BELT_MAX_FORWARD_VOLTAGE);}, () -> setBottom(0.0));
    }

    public Command beltBackward (){
        return this.runEnd(() -> { setTop(-FORWARD_VOLTAGE); setBottom(-FORWARD_VOLTAGE);}, () -> setBottom(0.0));
    }

    public Command beltStop (){
        return new InstantCommand(() -> setBottom(0.0));
    }

    public Command waitNote(boolean desiredState){
        return new Command() {
            boolean watching = true;
            @Override
            public void initialize() {
                if (hasNote() == desiredState){
                    watching = false;
                }
            }

            @Override
            public boolean isFinished() {
                return hasNote() == desiredState && watching;
            }
        };
    }

    public boolean hasNote(){
        return !beamBreak.get(); // beam break returns false on default state
    }
}
