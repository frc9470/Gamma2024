package com.team9470.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Consts.AmpevatorConstants;

public class Climber extends SubsystemBase {

    private static Climber instance;
    private final CANSparkMax elevator1 = new CANSparkMax(ClimberConstants.ID_1, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax elevator2 = new CANSparkMax(ClimberConstants.ID_2, CANSparkLowLevel.MotorType.kBrushless);

    private Climber(){
        elevator1.restoreFactoryDefaults();
        elevator2.restoreFactoryDefaults();
        elevator1.setInverted(ClimberConstants.INVERTED);
        elevator2.setInverted(ClimberConstants.INVERTED);

        elevator2.follow(elevator1, false);
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
        elevator1.setVoltage(voltage);
    }

    public Command prepareClimb(){
        return this.runEnd(() -> setVoltage(-AmpevatorConstants.CLIMB_VOLTAGE), () -> setVoltage(0.0));
    }

    public Command climb(){
        return this.runEnd(() -> setVoltage(AmpevatorConstants.CLIMB_VOLTAGE), () -> setVoltage(0.0));
    }
}
