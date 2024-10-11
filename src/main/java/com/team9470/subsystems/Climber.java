package com.team9470.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Consts.ClimberConstants;
import static com.team9470.Util.clamp;

public class Climber extends SubsystemBase {

    private static Climber instance;
    private final CANSparkMax climber1 = new CANSparkMax(ClimberConstants.ID_1, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax climber2 = new CANSparkMax(ClimberConstants.ID_2, CANSparkLowLevel.MotorType.kBrushless);


    private Climber(){
        climber1.restoreFactoryDefaults();
        climber2.restoreFactoryDefaults();
        climber1.setInverted(ClimberConstants.INVERTED);
        climber2.setInverted(ClimberConstants.INVERTED);

        climber2.follow(climber1, false);

    }

    public static Climber getInstance(){
        if(instance == null){
            instance = new Climber();
        }
        return instance;
    }

    @Override
    public void periodic() {

    }

    public void setVoltage(double voltage){
        climber1.setVoltage(voltage);
    }

    public Command prepareClimb(){
        return this.runEnd(() -> setVoltage(-ClimberConstants.CLIMB_VOLTAGE), () -> setVoltage(0.0));
    }

    public Command climb(){
        return this.runEnd(() -> setVoltage(ClimberConstants.CLIMB_VOLTAGE), () -> setVoltage(0.0));
    }
}
